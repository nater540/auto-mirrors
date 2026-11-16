//! A CAN 2.0 frame for the tests, standing in for the esp-hal TWAI frame: the `embedded-can` trait and nothing else.

use embedded_can::{Frame, Id, StandardId};

/// A frame as the bus would deliver it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TestFrame {
  id: Id,
  remote: bool,
  dlc: usize,
  data: [u8; 8],
}

impl TestFrame {
  /// A standard data frame. Panics on an identifier above 11 bits or a payload above eight bytes.
  pub fn standard(raw: u16, payload: &[u8]) -> Self {
    Self::new(StandardId::new(raw).unwrap(), payload).unwrap()
  }

  /// The 11-bit identifier. Panics on an extended frame.
  pub fn raw_id(&self) -> u16 {
    match self.id {
      Id::Standard(id) => id.as_raw(),
      Id::Extended(_) => panic!("an extended frame has no 11-bit identifier"),
    }
  }
}

impl Frame for TestFrame {
  fn new(id: impl Into<Id>, payload: &[u8]) -> Option<Self> {
    let mut data = [0; 8];
    data.get_mut(..payload.len())?.copy_from_slice(payload);
    Some(Self {
      id: id.into(),
      remote: false,
      dlc: payload.len(),
      data,
    })
  }

  fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
    (dlc <= 8).then(|| Self {
      id: id.into(),
      remote: true,
      dlc,
      data: [0; 8],
    })
  }

  fn is_extended(&self) -> bool {
    matches!(self.id, Id::Extended(_))
  }

  fn is_remote_frame(&self) -> bool {
    self.remote
  }

  fn id(&self) -> Id {
    self.id
  }

  fn dlc(&self) -> usize {
    self.dlc
  }

  fn data(&self) -> &[u8] {
    if self.remote { &[] } else { &self.data[..self.dlc] }
  }
}
