//! The first test of reference 6.5: the five memory reads once (steps 2 to 4), then, if asked, the status and the
//! position polled (step 4 while jogging from the panel, step 6 during a LightBurn upload from another computer).
//!
//! Queries go one at a time, each starting at least the poll period after the last, and each runs as a
//! [`ruida_proto::Exchange`]. Every datagram sent or received is an `RU1` line, with its wire bytes and what they
//! unswizzle to, so a pasted log shows what the controller did even where the probe could not read it. A datagram
//! from any address but the controller's is logged and ignored.

use std::fmt::Write as _;
use std::io::{self, Write};
use std::net::{SocketAddr, UdpSocket};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Context;
use ruida_proto::{
  ChecksumBasis, Datagram, DatagramError, Exchange, Limits, MAGIC_644XG, Outcome, Packet, Query, Step, decode_datagram,
  decode_u35, unswizzle
};

/// The first field of every line the probe writes.
pub const LOG_TAG: &str = "RU1";

/// What is polled after the five reads, in turn.
pub const POLLED: [Query; 3] = [Query::Status, Query::PositionX, Query::PositionY];

/// The largest datagram read whole; anything longer is truncated by the socket and logged as received.
const DATAGRAM_CAPACITY: usize = 2048;

/// How a probe runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Settings {
  /// Each query's timeout and tries.
  pub limits: Limits,
  /// Queries start at most this often.
  pub period_ms: u32,
  /// How long to poll [`POLLED`] after the five reads; zero for none.
  pub poll_ms: u64
}

/// How a probe went.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Summary {
  /// Queries that gave a reading.
  pub answered: u32,
  /// Queries that did not.
  pub failed: u32,
  /// The controller's checksum basis, once a packet was answered or acknowledged.
  pub basis: Option<ChecksumBasis>,
  /// Datagrams from the controller that answered nothing in flight: answers at other addresses, enquiries, and
  /// datagrams that could not be read.
  pub passed_over: u32,
  /// Datagrams from other addresses.
  pub foreign: u32
}

/// Run the probe against the controller at `controller` through `socket`, writing the log to `out`.
///
/// # Errors
///
/// A socket error other than a read timeout, or a failed write to `out`.
pub fn run(
  socket: &UdpSocket,
  controller: SocketAddr,
  settings: &Settings,
  out: &mut impl Write
) -> anyhow::Result<Summary> {
  let mut probe = Probe {
    socket,
    controller,
    limits: settings.limits,
    start: Instant::now(),
    summary: Summary::default(),
    out
  };
  writeln!(
    probe.out,
    "{LOG_TAG},start,controller={controller},local={},magic={MAGIC_644XG:#04x},\
     timeout_ms={},tries={},period_ms={},poll_ms={}",
    socket.local_addr().context("reading the probe's own address")?,
    settings.limits.timeout_ms,
    settings.limits.tries,
    settings.period_ms,
    settings.poll_ms
  )?;
  let period_ms = u64::from(settings.period_ms);
  let mut next_start_ms = 0;
  for query in Query::ALL {
    probe.wait_until(next_start_ms);
    next_start_ms = probe.now_ms().saturating_add(period_ms);
    probe.query(query)?;
  }
  let poll_until_ms = probe.now_ms().saturating_add(settings.poll_ms);
  for query in POLLED.into_iter().cycle() {
    probe.wait_until(next_start_ms);
    if probe.now_ms() >= poll_until_ms {
      break;
    }
    next_start_ms = probe.now_ms().saturating_add(period_ms);
    probe.query(query)?;
  }
  let summary = probe.summary;
  writeln!(
    probe.out,
    "{LOG_TAG},summary,answered={},failed={},basis={},passed_over={},foreign={}",
    summary.answered,
    summary.failed,
    summary.basis.map_or("unknown", ChecksumBasis::name),
    summary.passed_over,
    summary.foreign
  )?;
  probe.out.flush()?;
  Ok(summary)
}

/// One probe run in progress.
struct Probe<'a, W: Write> {
  socket: &'a UdpSocket,
  controller: SocketAddr,
  limits: Limits,
  start: Instant,
  summary: Summary,
  out: &'a mut W
}

impl<W: Write> Probe<'_, W> {
  /// Milliseconds since the run started.
  fn now_ms(&self) -> u64 {
    u64::try_from(self.start.elapsed().as_millis()).unwrap_or(u64::MAX)
  }

  fn wait_until(&self, at_ms: u64) {
    let now_ms = self.now_ms();
    if at_ms > now_ms {
      thread::sleep(Duration::from_millis(at_ms - now_ms));
    }
  }

  /// One query, from its packet to its reading or failure.
  fn query(&mut self, query: Query) -> anyhow::Result<()> {
    let (mut exchange, packet) = Exchange::start(query, self.summary.basis, self.limits, self.now_ms());
    self.send(packet, 1)?;
    let mut buffer = [0_u8; DATAGRAM_CAPACITY];
    let outcome = loop {
      let wait_ms = exchange.deadline_ms().saturating_sub(self.now_ms()).max(1);
      self.socket.set_read_timeout(Some(Duration::from_millis(wait_ms)))?;
      let step = match self.socket.recv_from(&mut buffer) {
        Ok((len, from)) => self.received(&mut exchange, buffer.get(..len).unwrap_or_default(), from)?,
        Err(error) if matches!(error.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => {
          exchange.on_time(self.now_ms())
        }
        // Windows reports an ICMP port unreachable on the next receive, even on an unconnected socket: worth a line
        // (reference 6.5 step 3 checks the address and port first), not the end of the run.
        Err(error)
          if matches!(
            error.kind(),
            io::ErrorKind::ConnectionReset | io::ErrorKind::ConnectionRefused
          ) =>
        {
          let now_ms = self.now_ms();
          writeln!(self.out, "{LOG_TAG},unreachable,t_ms={now_ms},error={error}")?;
          exchange.on_time(now_ms)
        }
        Err(error) => return Err(error).context("receiving from the controller")
      };
      match step {
        Step::Wait => {}
        Step::Send(packet) => self.send(packet, exchange.sends())?,
        Step::Done(outcome) => break outcome
      }
    };
    self.finished(&outcome)
  }

  fn received(&mut self, exchange: &mut Exchange, wire: &[u8], from: SocketAddr) -> anyhow::Result<Step> {
    let now_ms = self.now_ms();
    if from.ip() != self.controller.ip() {
      self.summary.foreign = self.summary.foreign.saturating_add(1);
      writeln!(
        self.out,
        "{LOG_TAG},foreign,t_ms={now_ms},from={from},wire={}",
        hex(wire.iter().copied())
      )?;
      return Ok(exchange.on_time(now_ms));
    }
    let decoded = decode_datagram(wire);
    writeln!(
      self.out,
      "{LOG_TAG},recv,t_ms={now_ms},from={from},wire={},plain={},meaning={}",
      hex(wire.iter().copied()),
      hex(wire.iter().map(|&byte| unswizzle(byte, MAGIC_644XG))),
      meaning(&decoded)
    )?;
    let passed_over = match decoded {
      Ok(Datagram::Answer { address, .. }) => address != exchange.query().address(),
      Ok(Datagram::Enq) | Err(_) => true,
      Ok(Datagram::Ack | Datagram::Nak | Datagram::Err) => false
    };
    if passed_over {
      self.summary.passed_over = self.summary.passed_over.saturating_add(1);
    }
    Ok(exchange.on_datagram(wire, now_ms))
  }

  fn send(&mut self, packet: Packet, try_number: u8) -> anyhow::Result<()> {
    self
      .socket
      .send_to(packet.bytes(), self.controller)
      .context("sending to the controller")?;
    writeln!(
      self.out,
      "{LOG_TAG},send,t_ms={},query={},try={try_number},basis={},wire={},plain={}",
      self.now_ms(),
      packet.query().name(),
      packet.basis().name(),
      hex(packet.bytes().iter().copied()),
      hex(packet.query().plain())
    )?;
    Ok(())
  }

  fn finished(&mut self, outcome: &Outcome) -> anyhow::Result<()> {
    if outcome.basis.is_some() {
      self.summary.basis = outcome.basis;
    }
    let now_ms = self.now_ms();
    let query = outcome.query.name();
    match outcome.result {
      Ok(reading) => {
        self.summary.answered = self.summary.answered.saturating_add(1);
        writeln!(
          self.out,
          "{LOG_TAG},reading,t_ms={now_ms},query={query},{reading},basis={},sends={}",
          outcome.basis.map_or("unknown", ChecksumBasis::name),
          outcome.sends
        )?;
      }
      Err(failure) => {
        self.summary.failed = self.summary.failed.saturating_add(1);
        writeln!(
          self.out,
          "{LOG_TAG},failed,t_ms={now_ms},query={query},reason={},sends={}",
          failure.name(),
          outcome.sends
        )?;
      }
    }
    Ok(())
  }
}

/// Bytes as two hex digits each, separated by spaces.
fn hex(bytes: impl IntoIterator<Item = u8>) -> String {
  let mut text = String::new();
  for byte in bytes {
    if !text.is_empty() {
      text.push(' ');
    }
    let _ = write!(text, "{byte:02x}");
  }
  text
}

/// What a datagram from the controller says, as `meaning=` and the fields after it.
fn meaning(decoded: &Result<Datagram, DatagramError>) -> String {
  match *decoded {
    Ok(Datagram::Ack) => "ack".into(),
    Ok(Datagram::Nak) => "nak".into(),
    Ok(Datagram::Err) => "err".into(),
    Ok(Datagram::Enq) => "enq".into(),
    Ok(Datagram::Answer { address, value, second }) => format!(
      "answer,address={},value={:#x},second={}",
      hex(address),
      decode_u35(value),
      second.map_or_else(|| "none".into(), |second| format!("{:#x}", decode_u35(second)))
    ),
    Err(DatagramError::Empty) => "unreadable,error=empty".into(),
    Err(DatagramError::UnknownByte { byte }) => format!("unreadable,error=unknown_byte,byte={byte:#04x}"),
    Err(DatagramError::Truncated { len }) => format!("unreadable,error=truncated,len={len}"),
    Err(DatagramError::NotData { offset, byte }) => {
      format!("unreadable,error=not_data,offset={offset},byte={byte:#04x}")
    }
    Err(DatagramError::Unrecognized { start, len }) => {
      format!("unreadable,error=unrecognized,start={},len={len}", hex(start))
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn bytes_are_written_as_spaced_lowercase_hex() {
    assert_eq!(hex([0x02, 0x61, 0xD4]), "02 61 d4");
    assert_eq!(hex([0x0A]), "0a");
    assert_eq!(hex([]), "");
  }

  #[test]
  fn each_datagram_is_given_its_meaning_and_fields() {
    assert_eq!(meaning(&Ok(Datagram::Ack)), "ack");
    assert_eq!(meaning(&Ok(Datagram::Nak)), "nak");
    assert_eq!(meaning(&Ok(Datagram::Err)), "err");
    assert_eq!(meaning(&Ok(Datagram::Enq)), "enq");
    assert_eq!(
      meaning(&Ok(Datagram::Answer {
        address: [0x05, 0x7E],
        value: [0x06, 0x28, 0x01, 0x4A, 0x00],
        second: None
      })),
      "answer,address=05 7e,value=0x65006500,second=none"
    );
    assert_eq!(
      meaning(&Ok(Datagram::Answer {
        address: [0x04, 0x21],
        value: [0, 0, 0, 0, 1],
        second: Some([0, 0, 0, 1, 0])
      })),
      "answer,address=04 21,value=0x1,second=0x80"
    );
    assert_eq!(meaning(&Err(DatagramError::Empty)), "unreadable,error=empty");
    assert_eq!(
      meaning(&Err(DatagramError::UnknownByte { byte: 0x55 })),
      "unreadable,error=unknown_byte,byte=0x55"
    );
    assert_eq!(
      meaning(&Err(DatagramError::Truncated { len: 7 })),
      "unreadable,error=truncated,len=7"
    );
    assert_eq!(
      meaning(&Err(DatagramError::NotData { offset: 4, byte: 0x80 })),
      "unreadable,error=not_data,offset=4,byte=0x80"
    );
    assert_eq!(
      meaning(&Err(DatagramError::Unrecognized {
        start: [0xD9, 0x00],
        len: 4
      })),
      "unreadable,error=unrecognized,start=d9 00,len=4"
    );
  }
}
