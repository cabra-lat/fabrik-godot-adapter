# GodotIK compatibility: what a hotswap would actually require

This is a compatibility assessment, not a claim of compatibility. Nothing here
has been integrated into the game, and no scene in this repository runs both
solvers.

Everything below was read from source, not from documentation summaries alone:

- GodotIK v1.3.1 (`/tmp/shooter/GodotIK-v1.3.1`): `src/godot_ik.cpp`,
  `src/godot_ik_effector.h`, `src/godot_ik_constraint.h`, `doc_classes/*.xml`.
  MIT licensed, © 2025 Alexander Montag (monxa) and contributors. No licence
  blocker.
- Godot 4.4's `extension_api.json` and godot-cpp's generated headers, for the
  `SkeletonModifier3D` / `Skeleton3D` surface the modifier is written against.
- The FABRIK papers, for what the reference algorithm does and does not
  constrain. See the core repository's `docs/ALGORITHM.md`.

One note on naming before comparing sources: GodotIK calls its tip-ward sweep
`solve_backward()` and its root-ward sweep `solve_forward()`, which is the
**opposite** of both FABRIK papers, where the tip-ward sweep is
"STAGE 1: FORWARD REACHING". The core follows the papers; GodotIK does not. The
two codebases agree on what happens, and disagree on what to call it.

## What GodotIK actually is

| Node | Base | Surface (read from source) |
| --- | --- | --- |
| `GodotIK` | `SkeletonModifier3D` | driven by the engine through `_process_modification()`; solves in **skeleton-local space**; `bone_idx`, `iteration_count` (default 8), `use_global_rotation_poles`; methods `get_bone_position`, `get_current_iteration`, `get_effectors`, `set_effector_transforms_to_bones` |
| `GodotIKEffector` | `Node` | `bone_name`, `bone_idx`, `chain_length`, `active`, `influence` (0–1), `transform_mode` ∈ POSITION_ONLY / PRESERVE_ROTATION / STRAIGHTEN_CHAIN / FULL_TRANSFORM |
| `GodotIKConstraint` | `Node` | abstract, must be a direct child of an effector; virtual `apply(pos_parent_bone, pos_bone, pos_child_bone, chain_direction) -> PackedVector3Array` in skeleton-local space, `chain_direction` ∈ FORWARD(+1) / BACKWARD(−1); `bone_idx`/`bone_name`; `get_ik_controller()`, `get_skeleton()` |
| `GodotIKRoot` | `Node` | `ik_controller` NodePath |

`GodotIK::initialize_chains()` builds every chain by walking
`Skeleton3D::get_bone_parent()` upward from the effector bone for
`chain_length` steps, and skips any chain whose bone has no parent. So a
GodotIK chain is **strictly tree-shaped**: there are no closed loops and no leaf
joints (a leaf joint being an end effector that also has a parent chain, which
the 2015 FABRIK paper handles and this code does not).

That is the single most useful fact for planning: **closed loops and leaf joints
are not parity requirements.** They are scope items relative to the literature,
not gaps relative to GodotIK. They drop off the critical path entirely.

## The gaps, in the order that blocks a substitution

**Status: items 1-4 and 7 are implemented and tested; 5, 6, 8 and 9 are not.**
The tests live in `demo/tests/test_modifier.gd` and run in CI.

1. ~~**No engine hook.**~~ **DONE.** `FabrikModifier3D : SkeletonModifier3D`
   overrides `_process_modification()` and solves on every skeleton update. The
   test that proves the *engine* calls it is the only one that never calls
   `solve_now()` itself.
2. ~~**No node-side effector.**~~ **DONE.** `FabrikEffector : Node3D` with
   `bone_name`, `chain_length`, `active`, `influence`, `transform_mode` and
   `pole_target_path`; the modifier discovers effector children in tree order
   and builds each chain by walking `get_bone_parent()` upwards, exactly as
   `initialize_chains()` does.
3. ~~`influence` is not `smoothing`.~~ **DONE.** Like GodotIK, influence is a
   per-chain blend from the leaf bone's current position towards the effector
   position, measured against the pose each frame, multiplied by the modifier's
   own inherited `influence`. `0` skips the chain outright. The test checks
   `0`, `0.5` and `1.0` as numbers, not as flags.
4. ~~`transform_mode` has no equivalent.~~ **DONE, all four.**
   `POSITION_ONLY`, `PRESERVE_ROTATION`, `STRAIGHTEN_CHAIN` and
   `FULL_TRANSFORM` are implemented with the same meaning as GodotIK's, and the
   test checks that each one differs from the others *and* that all four still
   solve the position - a mode that fixed the rotation by refusing to move
   anything would pass a rotation check and fail that one.
5. **The constraint extension point is a different shape.** *Not done.* GodotIK's
   extensibility is a per-bone virtual `apply(parent, bone, child, direction)`
   returning three positions, in skeleton-local space. The pole vector and angle
   limits here are post-solve projections, and a pole target is a
   `pole_target_path` rather than a child constraint node. Any project using
   custom GodotIK constraints loses them on a swap.
6. **Ancestor propagation and rest pose.** *Not done.* GodotIK keeps
   `initial_transforms` and propagates from a chain's ancestor
   (`closest_parent_in_chain`, `pivot_child_in_ancestor`) on every iteration.
   `FabrikModifier3D` has neither; chains are solved in effector declaration
   order and a bone moved by two chains is written twice, the second write
   winning. Overlapping chains will therefore diverge from GodotIK even though
   both are "correct" in isolation. Segment lengths are measured from the
   current pose, not the rest.
7. ~~**Space conversion.**~~ **DONE, and measured rather than assumed.** With
   the skeleton at `(10, 20, 30)`, `get_bone_global_pose()` still reports
   skeleton-space coordinates, so goals are converted in with
   `Skeleton3D.to_local()` and results out as
   `pose = parent_global.inverse() * new_global`, which was verified by writing a
   known global transform and reading it back. The test asserts the solved pose
   is identical for skeletons at three different transforms.
8. **Iteration defaults differ.** *Partly addressed.* `iteration_count` defaults
   to **8**, GodotIK's value rather than this adapter's 64, so a hotswap does
   not change the pose by changing a default. There is still no measurement of
   how much the two solvers' poses differ on the same skeleton.
9. **Tooling.** *Not done.* No scene converter, and no dual-run parity harness
   that runs both solvers on one skeleton and reports per-bone deltas. Without
   it, "hotswappable" remains a design claim rather than a measured one.

## What a GodotIK effector node actually needs

A `GodotIKEffector` is a `GodotIKEffector`, not a `FabrikEffector`. A GDExtension
cannot make its own class pass an `is_class("GodotIKEffector")` test, so a swap
means **retyping the effector nodes in the scene**, not renaming them. That is
the single largest piece of real migration work, and it is a scene-authoring
cost, not a code cost.

## Recommendation

Build 1 + 2 + 3 first: together they are the minimum for a scene to be able to
point at this adapter instead of GodotIK. Then 4 and 6 for behavioural parity.
Item 5 only if the consuming project actually uses custom constraints. Item 7
comes with item 1. The cheapest honest milestone is 1 + 2 + 3 plus a demo scene
that runs both solvers on one skeleton and prints the largest per-bone delta.

**Update: 1, 2, 3, 4 and 7 are done and covered by tests, and the suite is now
negative-controlled** (see below). The next useful work, in order: the dual-run
parity harness (9, which also settles 8), ancestor propagation (6), then a
`FabrikConstraint` with GodotIK's signature (5) if a consuming project needs it.
The scene-authoring cost of retyping effector nodes is unavoidable and should be
weighed before committing to a hotswap at all.

Keep all of it in this repository. Deciding whether the game takes a
backend-agnostic façade (and which one) is a project-level decision, and the
alternative to a hotswap — a straight replacement behind one interface — is worth
weighing explicitly, because a hotswap implies a scene that can hold both
implementations at once.

## The tests are negative-controlled

A characterization that only ever prints PASS is not evidence. Each of the three
critical paths in this adapter was disabled on a throwaway branch
(`negative-control`, never merged) and the suite had to go red. All three failed
on Ubuntu, macOS and Windows.

| Control | Change | Result |
| --- | --- | --- |
| A - engine hook | `_process_modification()` returns without solving | `FAIL engine-driven solve reported 0 chains, expected 1` (run `36193062241`) |
| B - space conversion | goal taken in world space instead of `Skeleton3D.to_local()` | 4 failures, all the space assertions, e.g. `FAIL skeleton offset changed the solved pose: (1.2, 0.6, 0.7) vs (0.588, 2.029, 1.611)` (run `36193400149`) |
| C - write order | poses written leaf-first instead of root-first | `FAIL transform mode 2 did not solve the position: off by 0.202` and `FAIL STRAIGHTEN_CHAIN left a rotation on the leaf pose` (run `36193817611`) |

Two honest notes on what the controls do and do not prove:

- **Control C is caught by the transform-mode tests, not by the length test.**
  Writing leaf-first leaves each child off by its parent's own delta, which is
  small on a unit-length fixture; the segment-length assertion still passes. The
  order is load-bearing and the suite does catch it, but only through the
  position assertions, so those are the ones to keep if the length test is ever
  trimmed.
- **The engine-hook test cannot read the bones back.** The engine resets bone
  poses to the base pose, runs the modifiers, and rebuilds the pose cache, so a
  modifier's `set_bone_pose()` is not visible through `get_bone_pose()`
  afterwards - measured, not assumed: a write made from *outside* the callback
  does stick, one made from inside does not. The test therefore asserts that the
  engine called the modifier and that the solve it reported is correct, via
  `get_last_bones()` / `get_last_solved_positions()`. It does not assert on
  `Skeleton3D` state, and it cannot.

## What these tests have already caught

The suite earned its keep before it ever went green:

- The chain root's forward direction was read from `solved[-1]`, because the code
  used `solved_index - 1` where the child of a bone at index `k` is `k + 1`. This
  segfaulted the engine-driven path; the crash was found by instrumenting
  `_solve_chain` and reading the log, not by reasoning.
- The solve report paired a root bone with the tip's position, so a correct solve
  was reported as a tip sitting on the root. The test caught it.
- The leaf bone was never rotated, which made `POSITION_ONLY` and
  `PRESERVE_ROTATION` the same code path. The test that the four modes differ
  caught it.
- `joint_limits` was bound as methods but never registered with
  `ADD_PROPERTY`, so `chain.joint_limits = ...` failed from GDScript while
  `set_joint_limits()` worked. Caught by the existing limits test's script error.
- A test of its own was wrong: it measured root-to-tip distance, which *must*
  shrink when a chain bends, and reported a correct solve as a length change. It
  now measures each adjacent bone pair.
