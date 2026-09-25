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

## Godot scene test

After building, copy the freshly built library into the descriptor's `bin/`
location, then run:

```sh
cp build/bin/libfabrik_adapter.so bin/libfabrik_adapter.so
godot --headless --path . --script res://tests/test_godot_scene.gd
```

The demo scene instantiates the registered `FabrikChain3D` RefCounted class and
solves a three-joint chain. The C++ smoke test separately exercises the C ABI's
success and unreachable statuses.

## API and limitations

- `FabrikChain3D` exposes packed 3D joints, optional segment lengths, target,
  root anchoring, tolerance, and iteration budget.
- The solver returns stable status integers: `0` success, `1` invalid input,
  `2` unreachable, `3` not converged, `4` degenerate chain.
- The core is a single-chain FABRIK implementation. Full-body graph ordering,
  closed loops, pole-vector constraints, collision, and runtime scene ownership
  are deliberately outside this prototype.
- Linux x86_64 is the tested host. The adapter workflow compiles and runs the C
  ABI smoke test on Linux, macOS, and Windows using each runner's native
  `gfortran`, CMake/Ninja, and `godot-cpp` toolchain; Linux additionally runs a
  Godot 4.4 scene parse/instantiate check. Windows/macOS extension runtime
  loading remains a native-toolchain validation item, not a claim of universal
  binary support. No cross-build or binary is committed here.
- CI caches the `godot-cpp` build with `ccache` (via
  `CMAKE_CXX_COMPILER_LAUNCHER`), keyed by OS, build type, and the pinned
  `godot-cpp` commit. `godot-cpp` 4.4-stable ships no CMake install/export
  rules, so there is no prebuilt package to `find_package()` and cache instead;
  the compiler cache is the one that composes with the `add_subdirectory()`
  build. The first run on a commit is a cold cache; later runs reuse it.
