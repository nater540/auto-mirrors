//! 28BYJ-48 geared stepper driver on a ULN2003, generic over `embedded_hal::digital::OutputPin`.
//!
//! Empty until milestone M4. The drive scheme (full-step, two-phase-on, coils off between moves, stop flag,
//! rail-lost handling) is `docs/ADDENDUM_A_28BYJ48.md` section A.4.
#![no_std]
