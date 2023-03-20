[![crates.io](https://img.shields.io/crates/d/arducam-mega.svg)][cratesio]
[![crates.io](https://img.shields.io/crates/v/arducam-mega.svg)][cratesio]
[![Documentation](https://docs.rs/arducam-mega/badge.svg)][docsrs]
![Minimum Supported Rust Version](https://img.shields.io/badge/rustc-1.68+-blue.svg)

[cratesio]: https://crates.io/crates/arducam-mega
[docsrs]: https://docs.rs/arducam-mega

# `arducam-mega`

`arducam-mega` is an [`embedded-hal`][hal] SPI driver for the [Arducam
Mega][mega].

[hal]: https://github.com/rust-embedded/embedded-hal
[mega]: https://www.arducam.com/camera-for-any-microcontroller/

## [API reference][docsrs]

## Status and device support

This crate is currently in the very early stages of development,
and should be considered experimental.

While this crate aims to provide support for both the 3MP and 5MP versions of
the Arducam Mega, it is only tested using the 5MP version.

We welcome testing using other cameras and other embedded systems.

## Minimum Supported Rust Version (MSRV)

This crate is guaranteed to compile on stable Rust 1.59 and up. It *might*
compile with older versions but that may change in any new patch release.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
