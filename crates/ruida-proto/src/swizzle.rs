//! The byte scrambling of reference 6.2 (EduTech Wiki, MeerK40t `rdjob.py`): swap bit 7 and bit 0, XOR the
//! magic, add 1. Every byte to and from the controller's main port is scrambled this way.

/// VERIFIED (reference 6.2: EduTech Wiki, MeerK40t): the magic of the RDC644XG family, to which the RDC6445G
/// belongs. It is the only magic this crate sends with (see the crate documentation).
pub const MAGIC_644XG: u8 = 0x88;

/// One byte as the controller expects it on the wire.
pub const fn swizzle(byte: u8, magic: u8) -> u8 {
  (swap_ends(byte) ^ magic).wrapping_add(1)
}

/// One byte from the wire as the controller meant it: the inverse of [`swizzle`] at the same magic.
pub const fn unswizzle(byte: u8, magic: u8) -> u8 {
  swap_ends(byte.wrapping_sub(1) ^ magic)
}

/// The byte with bit 7 and bit 0 exchanged, by the three XORs of reference 6.2.
const fn swap_ends(mut byte: u8) -> u8 {
  byte ^= byte >> 7;
  byte ^= byte << 7;
  byte ^= byte >> 7;
  byte
}

#[cfg(test)]
mod tests {
  use super::*;

  /// Every byte value, for the exhaustive checks.
  fn every_byte() -> impl Iterator<Item = u8> {
    0..=u8::MAX
  }

  #[test]
  fn the_controller_s_answers_arrive_as_reference_6_1_says_at_the_644xg_magic() {
    // Reference 6.0 and 6.1: ACK 0xCC arrives as 0xC6, ERR 0xCD as 0x46, NAK 0xCF as 0x48.
    assert_eq!(swizzle(0xCC, MAGIC_644XG), 0xC6);
    assert_eq!(swizzle(0xCD, MAGIC_644XG), 0x46);
    assert_eq!(swizzle(0xCF, MAGIC_644XG), 0x48);
    // The same algebra for ENQ 0xCE, which the reference does not work out.
    assert_eq!(swizzle(0xCE, MAGIC_644XG), 0xC8);
  }

  #[test]
  fn a_zero_byte_goes_out_as_the_magic_plus_one() {
    // MeerK40t finds a file's magic from this: zero is the commonest byte of a job.
    for magic in every_byte() {
      assert_eq!(swizzle(0, magic), magic.wrapping_add(1));
    }
  }

  #[test]
  fn only_bit_7_and_bit_0_trade_places() {
    assert_eq!(swap_ends(0b1000_0000), 0b0000_0001);
    assert_eq!(swap_ends(0b0000_0001), 0b1000_0000);
    assert_eq!(swap_ends(0b0111_1110), 0b0111_1110);
    assert_eq!(swap_ends(0b1000_0001), 0b1000_0001);
    assert_eq!(swap_ends(0b0101_1010), 0b0101_1010);
  }

  #[test]
  fn unswizzling_undoes_swizzling_for_every_byte_and_every_magic() {
    for magic in every_byte() {
      let mut seen = [false; 256];
      for byte in every_byte() {
        let wire = swizzle(byte, magic);
        assert_eq!(unswizzle(wire, magic), byte, "byte {byte:#04x} at magic {magic:#04x}");
        assert!(
          !seen[usize::from(wire)],
          "two bytes share the wire byte {wire:#04x} at magic {magic:#04x}"
        );
        seen[usize::from(wire)] = true;
      }
    }
  }

  #[test]
  fn a_byte_sent_at_one_magic_reads_at_another_as_itself_xor_their_difference_with_its_ends_swapped() {
    // This is how a wrong magic changes what the controller runs: by a fixed XOR on every byte.
    for sent in every_byte() {
      for read in every_byte() {
        let difference = swap_ends(sent ^ read);
        for byte in every_byte() {
          assert_eq!(unswizzle(swizzle(byte, sent), read), byte ^ difference);
        }
      }
    }
  }
}
