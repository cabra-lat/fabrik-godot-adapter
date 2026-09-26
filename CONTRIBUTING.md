# Contributing

## Scope

This repository is a **GDExtension adapter prototype**. It is intentionally
separate from the game and shooter repositories and is not integrated into
either. Please do not add game scenes, gameplay code or CI gates here.

## Ground rules

- The solver core lives in the separate `cabra-lat/fabrik-fortran` repository.
  This adapter may only consume it through the flat C ABI declared in
  `include/fabrik_core.h`. Fortran derived types, allocatables and descriptors
  must never cross that boundary.
- No binaries may be committed. `godot-cpp` is consumed as a pinned source
  checkout, and the extension library is always built by CI or by hand.
- Keep the demo Godot project inside `demo/`. A `project.godot` at the repository
  root makes the engine scan `build/`, `core/` and `godot-cpp/`.
- Attribute the algorithm; never copy it. See the provenance section of the
  core repository's licence review:
  <https://github.com/cabra-lat/fabrik-fortran/blob/16b3a7f8074eac5cbbd0eb7a3ad9d749a692369d/docs/LICENSE_REVIEW.md>

## Before you open a pull request

```sh
# 1. adapter build + C ABI smoke test
cmake -S . -B build -G Ninja \
  -DFABRIK_CORE_ROOT=<core install prefix> \
  -DGODOT_CPP_SOURCE_DIR=<godot-cpp source> \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure

# 2. headless Godot scene check (import once: a bare --script run on an
#    unimported project never registers .gdextension files)
mkdir -p demo/bin && cp build/bin/libfabrik_adapter.so demo/bin/
godot --headless --path demo --import
godot --headless --path demo --script res://tests/test_godot_scene.gd

# 3. hygiene
git diff --check
```

CI is the authority: a change is not done until the Linux, macOS and Windows
jobs are green. If you bump the core or the `godot-cpp` pin, update `flake.lock`,
`.github/workflows/ci.yml` and `README.md` together so they cannot drift.

## Commit and report conventions

Report what changed, the exact commands you ran, and the numbers you got. Never
report a gate as passing without its output.
