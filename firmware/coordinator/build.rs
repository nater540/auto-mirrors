//! Build script: adds the esp-hal linker script. Kept to the one line the image needs.

fn main() {
  // `linkall.x` must be the last linker script.
  println!("cargo:rustc-link-arg=-Tlinkall.x");
}
