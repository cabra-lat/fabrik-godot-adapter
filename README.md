# FABRIK Godot adapter prototype

This is a separate GDExtension adapter package. It consumes the standalone
`fabrik_core` package through its flat C ABI; it is not a production IK
replacement and is not integrated into the game or shooter addon.

## Reproducible source build

The adapter has no vendored binary dependency. Check out the pinned
`godot-cpp` `godot-4.4-stable` source tag and point CMake at the standalone
core install produced by the core flake:

```sh
core_root=$(nix build /path/to/fabrik-standalone#packages.x86_64-linux.default --no-link --print-out-paths)
git clone --branch godot-4.4-stable --depth 1 https://github.com/godotengine/godot-cpp.git /tmp/shooter/godot-cpp-4.4
# The CI lock is godot-cpp commit 714c9e2c165db2dcb7e6ea57e62a04204d3cfbfa.
cmake -S . -B build \
  -DFABRIK_CORE_ROOT="$core_root" \
  -DGODOT_CPP_SOURCE_DIR=/tmp/shooter/godot-cpp-4.4 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The resulting library is `build/bin/libfabrik_adapter.so`. The source checkout
is required rather than a downloaded opaque binary. The core Nix flake pins
its compiler and FPM inputs. CI pins `godot-cpp` to commit
`714c9e2c165db2dcb7e6ea57e62a04204d3cfbfa` from the `godot-4.4-stable` line.
The adapter workflow checks out the published
`cabra-lat/fabrik-fortran` repository at the reviewed commit pinned in
`.github/workflows/ci.yml`, so the separate core dependency cannot silently
drift.

## Runtime dependencies

The extension links the Fortran core, so it inherits the Fortran runtime:

| Platform | Dependency | Notes |
| --- | --- | --- |
| Linux | `libgfortran.so.5` | Must be on the loader path. A missing one surfaces only as `Can't open dynamic library: ... libgfortran.so.5`, not as anything mentioning Fortran. |
| macOS | system libgfortran | Present on GitHub runners and on any normal developer machine. |
| Windows | `libgfortran-5.dll`, shipped beside the DLL | A MinGW build imports it, and `-static-libgfortran` is best-effort - it does nothing on a toolchain with no `libgfortran.a`, which is the case on the CI runner. CI copies the runtime next to the extension and uploads the whole `demo/bin/` bundle. |

To reproduce a Linux run outside CI, point the loader at a toolchain that
provides it, for example:

```sh
LD_LIBRARY_PATH=/path/to/gfortran/lib godot --path demo --script res://tests/test_godot_scene.gd
```

A missing dependency on Windows surfaces only as
`Error 126: The specified module could not be found`, which names no DLL at
all. So CI prints the library's import table with `objdump` before the runtime
check: that is what identified `libgfortran-5.dll` as the cause, and it makes
any future load failure diagnosable from the log instead of guesswork.

## Visual demo

`demo/visual_demo.tscn` is a self-contained scene - one script, geometry built
in code, nothing to import - that drives a seven-joint chain after a moving
target and draws the bones and joints:

```sh
cp build/bin/libfabrik_adapter.so demo/bin/
godot --path demo res://visual_demo.tscn
```

Bones and joints are `MultiMesh` instances, so the chain costs two draw calls
and is posed directly from the rotations the adapter derives.

## Rendered humanoid demo

The humanoid demo solves six chains at once, drives the arm and leg targets
across their reach limits, and slowly circles the camera. The clip is uploaded
through GitHub's own attachment flow, so it plays inline here; the same file is
also published as a Release asset, and neither is committed to this source-only
repository:

<video src='https://github.com/user-attachments/assets/93853f53-15ba-4f92-b1c1-5017ee2a5f64' width='640'/>

[Download the MP4 (Release asset)](https://github.com/cabra-lat/fabrik-godot-adapter/releases/download/demo-v1/fabrik-humanoid-fixed-v4.mp4)

The inline copy lives on the attachment carrier issue
[#3](https://github.com/cabra-lat/fabrik-godot-adapter/issues/3); GitHub serves
both copies byte-identically
(`sha256 29e527f6951d0874163739a257d7f8b07dc1f1a5f6bccf3447cdbec66db780a6`).

The render is 1280×720, 24 FPS, 144 frames, and reproducible from the fixed
24 FPS scene runner.

## Godot scene test

After building, copy the freshly built library into the descriptor's `bin/`
location, then run:

```sh
mkdir -p demo/bin
cp build/bin/libfabrik_adapter.so demo/bin/libfabrik_adapter.so
# Import once: a bare --script run on an unimported project never registers
# .gdextension files, so the class would silently be "not declared".
godot --headless --path demo --import
godot --headless --path demo --script res://tests/test_godot_scene.gd
```

The Godot project lives in `demo/`, not at the repository root. A root-level
project would make the engine's filesystem scan walk `build/`, `core/` and
`godot-cpp/` (thousands of object files) on every start. The test script prints
the engine version, whether `FabrikChain3D` is registered, the contents of the
`.gdextension` descriptor, and the engine's loaded-extension list before it
asserts anything, so a failure identifies itself.

The demo scene instantiates the registered `FabrikChain3D` RefCounted class and
solves a three-joint chain. The C++ smoke test separately exercises the C ABI's
success and unreachable statuses.

## API and limitations

- `FabrikChain3D` exposes packed 3D joints, optional segment lengths, target,
  root anchoring, tolerance, iteration budget and a `smoothing` factor.
- `get_joint_rotations()` returns one quaternion per joint. The core is a
  *position* solver, so this is the adapter's job: each bone frame is parallel
  transported along the chain, which is what stops a bone from flipping as the
  chain bends. The convention is that a bone points along its local `+Y`,
  matching `Skeleton3D`.
- `pose_skeleton(skeleton, bone_names)` writes those rotations onto a
  `Skeleton3D` and returns the number of bones posed, or a negative code for
  invalid input (`-1` null skeleton, `-2` name/rotation count mismatch, `-3`
  unknown bone).
- `smoothing` interpolates each solve towards the previous pose, so a moving
  target eases instead of snapping every frame. `0` is the raw solve, `1`
  freezes the chain.
- A `solve_finished(status, residual)` signal fires on a successful solve.
- The solver returns stable status integers: `0` success, `1` invalid input,
  `2` unreachable, `3` not converged, `4` degenerate chain.
- A chain with a single segment cannot reach an arbitrary point inside its
  reach sphere; it correctly reports `NOT_CONVERGED` rather than claiming a
  solution. Multi-segment chains are the intended use.
- Stress measurements on the tested host: anchored root drift is exactly zero
  through 4096-joint chains, segment-length error stays below `2e-5` m at that
  size, and a 100,000-joint solve takes about 44 ms.
- `joint_limits` adds per-joint angle limits, given as the interior angle in
  degrees (`180` straight, `0` folded). FABRIK itself has no notion of them: the
  adapter projects them after the solve by rotating the sub-chain below the
  joint, which keeps an anchored root fixed and every segment length exact, and
  lets the tip fall short of the target instead.
  `get_limit_violation_count()` reports any joint the bounded relaxation could
  not satisfy, so a limit is never silently dropped. See
  [`docs/API.md`](docs/API.md).
- `FabrikRig3D` owns an ordered list of chains and solves them as one unit:
  declaration order is the default, `add_dependency(child, parent)` overrides
  it, and `set_target_provider(chain, callable)` derives a target from a chain
  solved earlier in the same frame. A dependency cycle is reported and the
  whole frame is refused rather than half-solved. See
  [`docs/API.md`](docs/API.md).
- The core is a single-chain FABRIK implementation. The adapter now supports an
  optional `pole_target` bend-plane constraint without changing segment lengths.
  Closed loops, collision, and runtime scene ownership remain outside this
  prototype: chains are still solved one at a time, and no constraint is
  projected across chains.
- Linux x86_64 is the tested host. The adapter workflow compiles and runs the C
  ABI smoke test on Linux, macOS, and Windows using each runner's native
  `gfortran`, CMake/Ninja, and `godot-cpp` toolchain; Linux additionally runs a
  Godot 4.4 scene parse/instantiate check. Windows/macOS extension runtime
  loading remains a native-toolchain validation item, not a claim of universal
  binary support. No cross-build or binary is committed here.
- CI loads the built extension in a real engine on **all three** runners and
  against **both Godot 4.4.1 and 4.7.1** - 4.7.1 is the version the game uses,
  so the matrix now covers the engine we actually ship. The built library is
  uploaded as a workflow artifact for manual inspection.
- `pole_target` rotates the solved intermediate joints around the root-to-tip
  axis, giving elbows and knees a deterministic side without stretching bones.
  It is a post-solve constraint, not a general dynamics or collision solver.
- CI caches the `godot-cpp` build with `ccache` (via
  `CMAKE_CXX_COMPILER_LAUNCHER`), keyed by OS, build type, and the pinned
  `godot-cpp` commit. `godot-cpp` 4.4-stable ships no CMake install/export
  rules, so there is no prebuilt package to `find_package()` and cache instead;
  the compiler cache is the one that composes with the `add_subdirectory()`
  build. The first run on a commit is a cold cache; later runs reuse it.
