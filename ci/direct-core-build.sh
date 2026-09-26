#!/usr/bin/env bash
# Build the core WITHOUT fpm, for platforms fpm publishes no binary for.
# fpm v0.13.0 ships linux-x86_64, macos-arm64, macos-x86_64 and
# windows-x86_64. There is no linux-arm64, and the release zip is the source
# repo with no bootstrap recipe, so building fpm itself is a nested toolchain
# build. The core is a handful of Fortran sources and one C file, so it is
# cheaper to compile it directly than to bootstrap a package manager.
#
# Deliberately NOT equivalent to the fpm build in every flag. -fPIC is added
# because the result is linked into a shared GDExtension; fpm adds it too, but
# being explicit here means a silent regression cannot reach the artifact.
set -euo pipefail

# Resolve BOTH arguments to absolute paths before any cd. The build runs in its
# own object directory, so a relative prefix would be resolved from there and
# `ar` would write to (or look for) core-prefix/lib/libfabrik_core.a under the
# object dir. Passing an absolute path from the caller hides that until CI,
# where the workflow naturally passes a workspace-relative one.
core_dir="$(cd "$1" && pwd)"
mkdir -p "$2"
prefix="$(cd "$2" && pwd)"
# Objects and test binaries go to a scratch directory, not into the repository:
# ci/obj/ would be untracked build litter that a later `git add -A` could sweep
# into a commit.
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

rm -rf "$work"
mkdir -p "$work" "$prefix/include" "$prefix/lib"
cd "$work"

echo "--- compiler"
gfortran --version | head -1

# Order the sources by their `use` statements, using a fixpoint: a source
# compiles once every module it needs is on disk. This is what fpm does from
# the same information; a per-file loop and a single `gfortran -c` both fail,
# because with -c gfortran processes files in argument order and does not
# resolve module dependencies (that only happens when building an executable).
# src/*.f90 in glob order puts consumers before providers, e.g.
# fabrik_c_api.f90 before the fabrik_core module it uses.
pending=("$core_dir"/src/*.f90)
passes=0
while [ "${#pending[@]}" -gt 0 ]; do
  still=()
  progressed=0
  for src in "${pending[@]}"; do
    if gfortran -O2 -fPIC -c "$src" 2>/dev/null; then
      progressed=1
    else
      still+=("$src")
    fi
  done
  pending=("${still[@]+"${still[@]}"}")
  passes=$((passes + 1))
  if [ "$progressed" -eq 0 ]; then
    echo "ERROR: cannot order these sources; a module is missing or cyclic:"
    printf '  %s\n' "${pending[@]}" >&2
    printf '  last compiler error for %s:\n' "${pending[0]}" >&2
    gfortran -O2 -fPIC -c "${pending[0]}" 2>&1 | head -5 >&2 || true
    exit 1
  fi
  if [ "$passes" -gt 20 ]; then
    echo "ERROR: source ordering did not converge after $passes passes" >&2
    exit 1
  fi
done
echo "--- ordered and compiled in $passes pass(es)"
gcc -O2 -fPIC -I "$core_dir"/include -c "$core_dir"/src/fabrik_status.c
ar rcs "$prefix/lib/libfabrik_core.a" ./*.o

# Both test files are standalone programs, so they run directly, linked
# against the objects just built. -I . finds the .mod files they use.
for t in test_fabrik_core test_pipeline; do
  echo "--- $t"
  gfortran -O2 -I . -o "$t" ./*.o "$core_dir"/test/"$t".f90
  "./$t"
done

cp "$core_dir"/include/fabrik_core.h "$prefix/include/"
echo "--- prefix assembled"
find "$prefix" -type f | sort
