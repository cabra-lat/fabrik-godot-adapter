# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- `FabrikChain3D` Godot class: packed 3D joints, optional segment lengths,
  target, root anchoring, tolerance and iteration budget, with stable status
  integers (`0` OK, `1` invalid input, `2` unreachable, `3` not converged,
  `4` degenerate chain).
- Standalone headless demo project under `demo/`, kept out of the repository
  root so the engine's filesystem scan never walks build artifacts.
- C ABI smoke test covering success, measured lengths, unreachable, degenerate,
  aliased input/output buffers and a null optional residual pointer.
- CI matrix on Linux, macOS and Windows, with a `ccache` build cache for the
  pinned `godot-cpp` checkout.

### Known limitations
- The core is a single-chain FABRIK implementation: no full-body graph ordering,
  closed loops, pole vectors, joint limits or collision handling.
- `FabrikChain3D` must be driven manually; there is no per-frame hook and the
  class is not `@tool`.
- Windows and macOS CI compile and run the C ABI smoke test, but only Linux
  loads the built extension in a Godot runtime.
