//! A node's bench console, driven as a person drives it: a command line out, and the node's lines back, of which only
//! the fixed lines of `node_logic::bench_line` are read. Every line both ways goes to the log.
//!
//! - **A line counts only if it arrived after the command it answers.** Lines already waiting are read and logged
//!   before a command goes out, so that an old move's end cannot answer a new one.
//! - **Nothing but `status` is sent until a `BENCH1,status` line has come back** ([`NodeConsole::status`]): that line
//!   is what proves a node of this firmware is on the port, not the coordinator, the sensor, or another board.
//! - **A move that does not end in time is stopped**: any key stops a bench move, so `stop` goes out, and the end it
//!   then reports is still read. A move with no end at all is [`ConsoleError::Silent`], and the motor's count is not
//!   known.

use std::cell::RefCell;
use std::io::{self, ErrorKind, Read, Write};
use std::rc::Rc;
use std::time::{Duration, Instant};

use node_logic::bench_line::{BenchEnd, BenchLine};
use node_logic::{BenchRefused, NodeState};

/// Where every line both ways is written, shared by the consoles and the readings.
pub type Log = Rc<RefCell<dyn Write>>;

/// How long a node has to answer `status`, each time it is asked.
pub const STATUS_WAIT: Duration = Duration::from_secs(2);

/// How many times `status` is asked before the node is taken to be absent. Opening the port may reset the board,
/// whose boot takes about a second.
pub const STATUS_TRIES: u32 = 5;

/// The longest a console reads what was already waiting before a command. A node that never stops printing would
/// otherwise keep the command from ever going out.
const DRAIN_MAX: Duration = Duration::from_millis(200);

/// The longest line kept; a longer one is cut there. The node's lines are under 200 characters.
const LINE_MAX: usize = 1_024;

/// Why the console could not do what was asked.
#[derive(Debug, thiserror::Error)]
pub enum ConsoleError {
  /// Reading from or writing to the port failed.
  #[error("the port of {node} failed: {source}")]
  Io {
    /// The node, as the command line named its port.
    node: String,
    /// What failed.
    source: io::Error
  },
  /// The node did not answer in time.
  #[error("{node} did not report {waiting_for} within {waited_ms} ms")]
  Silent {
    /// The node, as the command line named its port.
    node: String,
    /// What was awaited.
    waiting_for: &'static str,
    /// How long.
    waited_ms: u128
  }
}

/// What `status` reports of a node and one of its channels.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeStatus {
  /// The node's state.
  pub state: NodeState,
  /// The motor permit is present.
  pub permit: bool,
  /// The motor rail is up.
  pub rail: bool,
  /// The channel's count, in full steps from its commissioning position.
  pub pos_steps: i32,
  /// The count is not trusted.
  pub suspect: bool
}

/// How a jog ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Jog {
  /// The move ran, completely or not, and the node counted it.
  Moved {
    /// How it ended.
    end: BenchEnd,
    /// The node's count after it.
    pos_steps: i32,
    /// The count is not trusted.
    suspect: bool
  },
  /// The node refused it: nothing moved.
  Refused(BenchRefused)
}

/// One node's console on a port: anything that reads and writes bytes, and whose read gives up with
/// `ErrorKind::TimedOut` or `ErrorKind::WouldBlock` when nothing comes.
pub struct NodeConsole<P> {
  port: P,
  node: String,
  log: Log,
  pending: Vec<u8>,
  identified: bool
}

impl<P: Read + Write> NodeConsole<P> {
  /// A console on `port`, named `node` in the log and in errors.
  pub fn new(port: P, node: impl Into<String>, log: Log) -> Self {
    Self {
      port,
      node: node.into(),
      log,
      pending: Vec::new(),
      identified: false
    }
  }

  /// The node's name, as the command line gave its port.
  pub fn node(&self) -> &str {
    &self.node
  }

  /// Asks for the node's state and `channel`'s count, as many as [`STATUS_TRIES`] times.
  ///
  /// # Errors
  ///
  /// [`ConsoleError::Silent`] when no node of this firmware answers, or `channel` is not on it.
  pub fn status(&mut self, channel: u8) -> Result<NodeStatus, ConsoleError> {
    let mut last = None;
    for _ in 0..STATUS_TRIES {
      match self.status_once(channel) {
        Ok(status) => return Ok(status),
        Err(error @ ConsoleError::Silent { .. }) => last = Some(error),
        Err(error) => return Err(error)
      }
    }
    Err(last.unwrap_or_else(|| self.silent("its status", Duration::ZERO)))
  }

  fn status_once(&mut self, channel: u8) -> Result<NodeStatus, ConsoleError> {
    self.send_unchecked("status")?;
    let (state, permit, rail) = self.expect("its status", STATUS_WAIT, |line| match line {
      BenchLine::Status { state, permit, rail } => Some((state, permit, rail)),
      _ => None
    })?;
    self.identified = true;
    let (pos_steps, suspect) = self.expect("the channel's count", STATUS_WAIT, |line| match line {
      BenchLine::Channel {
        channel: reported,
        pos_steps,
        suspect
      } if reported == channel => Some((pos_steps, suspect)),
      _ => None
    })?;
    Ok(NodeStatus {
      state,
      permit,
      rail,
      pos_steps,
      suspect
    })
  }

  /// Enters or leaves bench mode. The node's state, read with [`NodeConsole::status`] afterward, says whether it did.
  ///
  /// # Errors
  ///
  /// [`ConsoleError::Silent`] before the node has answered `status`, or a failed port.
  pub fn bench(&mut self, on: bool) -> Result<(), ConsoleError> {
    self.send(if on { "bench on" } else { "bench off" })
  }

  /// Moves `channel` by `steps` at the node's configured rate and waits up to `wait` for the move's end. A move that
  /// has not ended by then is stopped, and its end is read for one [`STATUS_WAIT`] more.
  ///
  /// # Errors
  ///
  /// [`ConsoleError::Silent`] when the node never reports the move's end: its count is then not known.
  pub fn jog(&mut self, channel: u8, steps: i32, wait: Duration) -> Result<Jog, ConsoleError> {
    self.send(&format!("jog {channel} {steps}"))?;
    let end = |line| match line {
      BenchLine::Moved {
        channel: reported,
        end,
        pos_steps,
        suspect
      } if reported == channel => Some(Jog::Moved {
        end,
        pos_steps,
        suspect
      }),
      BenchLine::Refused {
        channel: reported,
        reason
      } if reported == channel => Some(Jog::Refused(reason)),
      _ => None
    };
    match self.expect("the move's end", wait, end) {
      Err(ConsoleError::Silent { .. }) => {
        // Any key stops a bench move; the command after it is for a move that already ended.
        self.write_line("stop")?;
        self.expect("the stopped move's end", STATUS_WAIT, end)
      }
      other => other
    }
  }

  /// Sends a command other than `status`, once the node has identified itself.
  fn send(&mut self, command: &str) -> Result<(), ConsoleError> {
    if !self.identified {
      return Err(self.silent("its status before any other command", Duration::ZERO));
    }
    self.send_unchecked(command)
  }

  fn send_unchecked(&mut self, command: &str) -> Result<(), ConsoleError> {
    self.drain()?;
    self.write_line(command)
  }

  fn write_line(&mut self, command: &str) -> Result<(), ConsoleError> {
    self.note(">", command);
    // A bare line feed: a carriage return before it would end the line, and the feed would then reach a running move.
    let written = self
      .port
      .write_all(format!("{command}\n").as_bytes())
      .and_then(|()| self.port.flush());
    written.map_err(|source| self.io(source))
  }

  /// Reads and logs what is already waiting, so that no answer to an earlier command is taken for the next one's.
  fn drain(&mut self) -> Result<(), ConsoleError> {
    let deadline = Instant::now() + DRAIN_MAX;
    while self.next_line(Instant::now())?.is_some() && Instant::now() < deadline {}
    self.pending.clear();
    Ok(())
  }

  /// Reads lines until `pick` accepts one, or `wait` passes, whether or not other lines keep arriving.
  fn expect<T>(
    &mut self,
    waiting_for: &'static str,
    wait: Duration,
    mut pick: impl FnMut(BenchLine) -> Option<T>
  ) -> Result<T, ConsoleError> {
    let deadline = Instant::now() + wait;
    loop {
      match self.next_line(deadline)? {
        Some(text) => {
          if let Some(found) = BenchLine::parse(&text).and_then(&mut pick) {
            return Ok(found);
          }
          if Instant::now() >= deadline {
            return Err(self.silent(waiting_for, wait));
          }
        }
        None => return Err(self.silent(waiting_for, wait))
      }
    }
  }

  /// The next whole line, logged, or `None` once `deadline` has passed with no line complete.
  fn next_line(&mut self, deadline: Instant) -> Result<Option<String>, ConsoleError> {
    loop {
      if let Some(end) = self.pending.iter().position(|byte| *byte == b'\n') {
        let bytes: Vec<u8> = self.pending.drain(..=end).collect();
        let text = String::from_utf8_lossy(&bytes).trim_end().to_owned();
        self.note("<", &text);
        return Ok(Some(text));
      }
      let mut chunk = [0_u8; 256];
      match self.port.read(&mut chunk) {
        Ok(0) => {}
        Ok(count) => {
          self.pending.extend(chunk.iter().take(count));
          if self.pending.len() > LINE_MAX && !self.pending.contains(&b'\n') {
            self.pending.truncate(LINE_MAX);
            self.pending.push(b'\n');
          }
          continue;
        }
        Err(error)
          if matches!(
            error.kind(),
            ErrorKind::TimedOut | ErrorKind::WouldBlock | ErrorKind::Interrupted
          ) => {}
        Err(source) => return Err(self.io(source))
      }
      if Instant::now() >= deadline {
        return Ok(None);
      }
    }
  }

  fn note(&self, direction: &str, text: &str) {
    // The log is a record for the owner; a failed write to it must not stop a motor mid-procedure.
    let _ = writeln!(self.log.borrow_mut(), "{} {direction} {text}", self.node);
  }

  fn io(&self, source: io::Error) -> ConsoleError {
    ConsoleError::Io {
      node: self.node.clone(),
      source
    }
  }

  fn silent(&self, waiting_for: &'static str, waited: Duration) -> ConsoleError {
    ConsoleError::Silent {
      node: self.node.clone(),
      waiting_for,
      waited_ms: waited.as_millis()
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::fake_node::{FakeNode, Machine};

  fn console(machine: &Machine) -> (NodeConsole<crate::fake_node::Port>, Rc<RefCell<Vec<u8>>>) {
    let log = Rc::new(RefCell::new(Vec::new()));
    let port = FakeNode::spawn(machine.clone());
    (NodeConsole::new(port, "node-a", log.clone()), log)
  }

  fn logged(log: &Rc<RefCell<Vec<u8>>>) -> String {
    String::from_utf8_lossy(&log.borrow()).into_owned()
  }

  #[test]
  fn the_status_is_read_from_the_fixed_lines_among_the_ones_for_people() {
    let machine = Machine::default();
    machine.set_count(0, -120);
    let (mut console, log) = console(&machine);
    let status = console.status(0).unwrap();
    assert_eq!(
      status,
      NodeStatus {
        state: NodeState::Idle,
        permit: true,
        rail: true,
        pos_steps: -120,
        suspect: false
      }
    );
    let log = logged(&log);
    assert!(log.contains("node-a > status"), "{log}");
    assert!(log.contains("node-a < INFO - BENCH1,status,idle,1,1"), "{log}");
  }

  #[test]
  fn a_node_that_misses_the_first_status_while_it_boots_is_asked_again() {
    let machine = Machine::default();
    machine.ignore_next(2);
    let (mut console, _) = console(&machine);
    assert_eq!(console.status(0).unwrap().pos_steps, 0);
  }

  #[test]
  fn a_port_where_no_node_of_this_firmware_answers_is_silent_and_nothing_else_is_sent() {
    let machine = Machine::default();
    machine.ignore_next(u32::MAX);
    let (mut console, log) = console(&machine);
    assert!(matches!(console.status(0), Err(ConsoleError::Silent { .. })));
    assert!(matches!(
      console.jog(0, 10, STATUS_WAIT),
      Err(ConsoleError::Silent { .. })
    ));
    assert!(matches!(console.bench(true), Err(ConsoleError::Silent { .. })));
    let sent: Vec<String> = machine.received();
    assert!(sent.iter().all(|line| line == "status"), "{sent:?}");
    assert!(!logged(&log).contains("jog"));
  }

  #[test]
  fn a_channel_the_node_does_not_have_is_silent() {
    let (mut console, _) = console(&Machine::default());
    assert!(matches!(console.status(3), Err(ConsoleError::Silent { .. })));
  }

  #[test]
  fn a_jog_ends_with_the_node_s_count_or_its_refusal() {
    let machine = Machine::default();
    let (mut console, _) = console(&machine);
    console.status(0).unwrap();
    assert_eq!(
      console.jog(0, 40, STATUS_WAIT).unwrap(),
      Jog::Refused(BenchRefused::NotInBench),
      "a node starts outside bench mode"
    );
    console.bench(true).unwrap();
    assert_eq!(console.status(0).unwrap().state, NodeState::Bench);
    assert_eq!(
      console.jog(0, 40, STATUS_WAIT).unwrap(),
      Jog::Moved {
        end: BenchEnd::Done,
        pos_steps: 40,
        suspect: false
      }
    );
    assert_eq!(
      console.jog(0, -55, STATUS_WAIT).unwrap(),
      Jog::Moved {
        end: BenchEnd::Done,
        pos_steps: -15,
        suspect: false
      }
    );
    assert_eq!(machine.received().last().map(String::as_str), Some("jog 0 -55"));
  }

  #[test]
  fn an_old_move_s_end_does_not_answer_a_new_one() {
    let machine = Machine::default();
    let (mut console, _) = console(&machine);
    console.status(0).unwrap();
    console.bench(true).unwrap();
    machine.echo_stale_end(0, 999);
    assert_eq!(
      console.jog(0, 5, STATUS_WAIT).unwrap(),
      Jog::Moved {
        end: BenchEnd::Done,
        pos_steps: 5,
        suspect: false
      }
    );
  }

  #[test]
  fn a_move_that_does_not_end_in_time_is_stopped_and_its_end_still_read() {
    let machine = Machine::default();
    let (mut console, _) = console(&machine);
    console.status(0).unwrap();
    console.bench(true).unwrap();
    machine.hold_moves_until_stopped();
    assert_eq!(
      console.jog(0, 100, Duration::from_millis(200)).unwrap(),
      Jog::Moved {
        end: BenchEnd::Stopped,
        pos_steps: 50,
        suspect: false
      }
    );
    assert_eq!(machine.received().last().map(String::as_str), Some("stop"));
  }

  /// A node that prints a line for people on every read and never a fixed line.
  struct Chatter;

  impl Read for Chatter {
    fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
      let line = b"INFO - chatter\n";
      let count = buffer.len().min(line.len());
      buffer[..count].copy_from_slice(&line[..count]);
      Ok(count)
    }
  }

  impl Write for Chatter {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
      Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
      Ok(())
    }
  }

  fn chatter() -> NodeConsole<Chatter> {
    // The log is discarded: an endless stream would otherwise fill it.
    let log: Log = Rc::new(RefCell::new(io::sink()));
    NodeConsole::new(Chatter, "node-a", log)
  }

  #[test]
  fn a_node_that_never_stops_printing_other_lines_is_still_silent_once_the_wait_has_passed() {
    let mut console = chatter();
    let started = Instant::now();
    let answer = console.expect("a fixed line", Duration::from_millis(100), |_| None::<()>);
    assert!(matches!(answer, Err(ConsoleError::Silent { .. })), "{answer:?}");
    assert!(started.elapsed() < Duration::from_secs(2), "{:?}", started.elapsed());
  }

  #[test]
  fn reading_what_is_waiting_ends_even_while_the_node_keeps_printing() {
    let mut console = chatter();
    let started = Instant::now();
    console.drain().unwrap();
    assert!(started.elapsed() < Duration::from_secs(2), "{:?}", started.elapsed());
  }
}
