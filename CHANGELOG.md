# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- `get_joint_rotations()`: per-joint bone orientations derived by parallel
  transport, so a bending chain no longer flips a bone frame.
- `pose_skeleton(skeleton, bone_names)`: apply those rotations to a
  `Skeleton3D`, with explicit negative codes for invalid input.
- `smoothing` property: per-solve interpolation towards the previous pose for
  temporal coherence when solving every frame.
- `solve_finished(status, residual)` signal.
- `demo/visual_demo.tscn`: self-contained animated chain, MultiMesh bones.
- Repository hygiene: MIT `LICENSE`, `CHANGELOG.md`, `CONTRIBUTING.md`,
  dependabot.
- Headless test coverage for rotations, smoothing and skeleton posing.
- CI now loads the extension on Linux, macOS **and** Windows, against Godot
  4.4.1 and 4.7.1, and uploads the built library as an artifact.
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
- `pose_skeleton` writes bone pose rotations directly; it does not compensate
  for non-identity rest poses, so a rig whose bones are not authored along
  local `+Y` needs the rest orientation folded in by the caller.
- There is no pole vector, joint limit or collision handling yet.
