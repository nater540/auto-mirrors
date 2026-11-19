//! A node's bench console for the tests, on a thread behind a port of channels: it parses commands with the node's
//! own parser (`node_logic::console`), answers with the node's own fixed lines (`node_logic::bench_line`) behind a
//! logger's prefix and colors as esp-println writes them ([`logged`]), and moves one motor on channel 0, whose screw follows it through a play. The dot on the target
//! follows the screw.

use std::io::{self, ErrorKind, Read, Write};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread;
use std::time::Duration;

use node_logic::bench_line::{BenchEnd, BenchLine};
use node_logic::console::{self, Command};
use node_logic::{BenchRefused, NodeState};

/// How long a read of the port waits, as a serial port's read timeout does.
const READ_WAIT: Duration = Duration::from_millis(20);

/// The machine behind the fake console, shared with the test that drives it.
#[derive(Clone, Default)]
pub struct Machine(Arc<Mutex<State>>);

#[derive(Default)]
struct State {
  bench: bool,
  count_steps: i32,
  /// Where the screw is, in the motor's steps: within `play_steps` behind the motor.
  screw_steps: i32,
  play_steps: i32,
  /// How far the dot moves on the target for a step of the screw.
  mm_per_step: f32,
  ignore_status: u32,
  hold_moves: bool,
  held: Option<i32>,
  /// Whether a move's answer is preceded by an end and a refusal for another channel.
  other_channel_first: bool,
  received: Vec<String>,
  to_host: Option<Sender<Vec<u8>>>
}

impl Machine {
  /// A machine whose screw has `play_steps` of play and moves the dot `mm_per_step` a step.
  pub fn with_play(play_steps: i32, mm_per_step: f32) -> Self {
    let machine = Self::default();
    {
      let mut state = machine.state();
      state.play_steps = play_steps;
      state.mm_per_step = mm_per_step;
    }
    machine
  }

  fn state(&self) -> MutexGuard<'_, State> {
    self.0.lock().unwrap_or_else(PoisonError::into_inner)
  }

  /// Sets channel 0's count, and puts the screw there.
  pub fn set_count(&self, channel: u8, count_steps: i32) {
    assert_eq!(channel, 0, "the fake node has channel 0 only");
    let mut state = self.state();
    state.count_steps = count_steps;
    state.screw_steps = count_steps;
  }

  /// Ignores the next `count` status commands, as a board still booting does.
  pub fn ignore_next(&self, count: u32) {
    self.state().ignore_status = count;
  }

  /// Runs no move to its end until a `stop` arrives; the stopped move has run half its steps.
  pub fn hold_moves_until_stopped(&self) {
    self.state().hold_moves = true;
  }

  /// Precedes the answer to every move with an end and a refusal on channel 1, as a node with two channels might.
  pub fn report_another_channel_first(&self) {
    self.state().other_channel_first = true;
  }

  /// Sends a move's end now, as if an earlier move had just ended.
  pub fn echo_stale_end(&self, channel: u8, pos_steps: i32) {
    let line = BenchLine::Moved {
      channel,
      end: BenchEnd::Done,
      pos_steps,
      suspect: false
    };
    if let Some(to_host) = &self.state().to_host {
      let _ = to_host.send(logged(&line.to_string()));
    }
  }

  /// Every line the node received, in order.
  pub fn received(&self) -> Vec<String> {
    self.state().received.clone()
  }

  /// Where the dot is on the target: horizontal and vertical, in millimeters.
  pub fn dot_mm(&self) -> (f32, f32) {
    let state = self.state();
    #[expect(clippy::cast_precision_loss, reason = "a few thousand steps")]
    let along = state.screw_steps as f32 * state.mm_per_step;
    (along, 0.3 * along)
  }
}

impl State {
  fn run(&mut self, steps: i32) {
    self.count_steps = self.count_steps.saturating_add(steps);
    let behind = self.count_steps.saturating_sub(self.play_steps);
    self.screw_steps = self.screw_steps.clamp(behind, self.count_steps);
  }

  fn answer(&mut self, line: &str) -> Vec<String> {
    self.received.push(line.to_owned());
    if line == "stop" {
      return match self.held.take() {
        Some(steps) => {
          self.run(steps / 2);
          vec![self.moved(BenchEnd::Stopped)]
        }
        None => Vec::new()
      };
    }
    match console::parse(line) {
      Ok(Command::Status) if self.ignore_status > 0 => {
        self.ignore_status -= 1;
        Vec::new()
      }
      Ok(Command::Status) => {
        let state = if self.bench { NodeState::Bench } else { NodeState::Idle };
        vec![
          format!("state {state:?}, permit true, rail true"),
          BenchLine::Status {
            state,
            permit: true,
            rail: true
          }
          .to_string(),
          format!("channel 0: count {}", self.count_steps),
          BenchLine::Channel {
            channel: 0,
            pos_steps: self.count_steps,
            suspect: false
          }
          .to_string(),
        ]
      }
      Ok(Command::Bench(on)) => {
        self.bench = on;
        vec![format!("bench mode {}", if on { "on" } else { "off" })]
      }
      Ok(Command::Jog { channel, steps, .. }) => {
        let refusal = if !self.bench {
          Some(BenchRefused::NotInBench)
        } else if channel != 0 {
          Some(BenchRefused::ChannelUnavailable { channel })
        } else {
          None
        };
        match refusal {
          Some(reason) => vec![BenchLine::Refused { channel, reason }.to_string()],
          None if self.hold_moves => {
            self.held = Some(steps);
            Vec::new()
          }
          None => {
            self.run(steps);
            let mut lines = Vec::new();
            if self.other_channel_first {
              let other = 1;
              let moved = BenchLine::Moved {
                channel: other,
                end: BenchEnd::Done,
                pos_steps: 999,
                suspect: false
              };
              let refused = BenchLine::Refused {
                channel: other,
                reason: BenchRefused::ChannelUnavailable { channel: other }
              };
              lines.extend([moved.to_string(), refused.to_string()]);
            }
            lines.push(self.moved(BenchEnd::Done));
            lines
          }
        }
      }
      _ => vec![format!("unknown: {line}")]
    }
  }

  fn moved(&self, end: BenchEnd) -> String {
    BenchLine::Moved {
      channel: 0,
      end,
      pos_steps: self.count_steps,
      suspect: false
    }
    .to_string()
  }
}

/// A line as the node's logger writes it: esp-println's default `colors` feature wraps every line in a terminal
/// color and a reset, so the fixed lines never end at the line's end (found on the bench, 2026-11-18).
pub fn logged(text: &str) -> Vec<u8> {
  format!("\u{1b}[32mINFO - {text}\u{1b}[0m\n").into_bytes()
}

/// The host's end of the fake console.
pub struct Port {
  to_node: Sender<Vec<u8>>,
  from_node: Receiver<Vec<u8>>,
  waiting: Vec<u8>
}

impl Read for Port {
  fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
    if self.waiting.is_empty() {
      match self.from_node.recv_timeout(READ_WAIT) {
        Ok(bytes) => self.waiting = bytes,
        Err(RecvTimeoutError::Timeout) => return Err(ErrorKind::TimedOut.into()),
        Err(RecvTimeoutError::Disconnected) => return Ok(0)
      }
    }
    let count = buffer.len().min(self.waiting.len());
    buffer[..count].copy_from_slice(&self.waiting[..count]);
    self.waiting.drain(..count);
    Ok(count)
  }
}

impl Write for Port {
  fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
    self
      .to_node
      .send(bytes.to_vec())
      .map_err(|_| io::Error::from(ErrorKind::BrokenPipe))?;
    Ok(bytes.len())
  }

  fn flush(&mut self) -> io::Result<()> {
    Ok(())
  }
}

/// The node's thread.
pub struct FakeNode;

impl FakeNode {
  /// Starts a fake node on `machine` and returns the host's end of its console.
  pub fn spawn(machine: Machine) -> Port {
    let (to_node, node_input) = mpsc::channel::<Vec<u8>>();
    let (to_host, from_node) = mpsc::channel::<Vec<u8>>();
    machine.state().to_host = Some(to_host.clone());
    thread::spawn(move || {
      let mut line = Vec::new();
      while let Ok(bytes) = node_input.recv() {
        for byte in bytes {
          if byte != b'\n' {
            line.push(byte);
            continue;
          }
          let text = String::from_utf8_lossy(&line).into_owned();
          line.clear();
          let answers = machine.state().answer(&text);
          for answer in answers {
            if to_host.send(logged(&answer)).is_err() {
              return;
            }
          }
        }
      }
    });
    Port {
      to_node,
      from_node,
      waiting: Vec::new()
    }
  }
}
