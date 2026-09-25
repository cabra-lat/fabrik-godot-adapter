# GodotIK compatibility: what a hotswap would actually require

This is a compatibility assessment, not a claim of compatibility. Nothing here
has been integrated into the game, and no scene in this repository runs both
solvers.

Everything below was read from source, not from documentation summaries alone:

- GodotIK v1.3.1 (`/tmp/shooter/GodotIK-v1.3.1`): `src/godot_ik.cpp`,
  `src/godot_ik_effector.h`, `src/godot_ik_constraint.h`, `doc_classes/*.xml`.
  MIT licensed, © 2025 Alexander Montag (monxa) and contributors. No licence
  blocker.
- The FABRIK papers, for what the reference algorithm does and does not
  constrain. See the core repository's `docs/ALGORITHM.md`.

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

1. **No engine hook.** This adapter is `RefCounted` plus a static
   `pose_skeleton()`. GodotIK is a `SkeletonModifier3D` the engine calls. A
   scene holding a `GodotIK` node cannot be pointed at this adapter at all, so
   nothing else matters until there is a `FabrikModifier3D : SkeletonModifier3D`.
   This is also the "runtime scene ownership" limitation listed in the README.
2. **No node-side effector.** `FabrikRig3D` moves the *data* (chains plus an
   optional target provider), but the substitutable unit in a GodotIK scene is a
   *node* carrying `bone_name` / `active` / `influence` / `transform_mode` /
   `chain_length`, discovered from the tree. Needed: a `FabrikEffector` node and
   chain construction from bone names.
3. **`influence` is not `smoothing`.** GodotIK lerps each chain's effector
   position between the current pose and the goal, by a per-chain weight that
   may change every frame (`global_pose_pos.lerp(effector_position, influence)`).
   `smoothing` is a temporal ease. Different mechanism, different feel; not
   substitutable until an equivalent exists.
4. **`transform_mode` has no equivalent.** This adapter does
   position-solve-then-derive-rotations. POSITION_ONLY, PRESERVE_ROTATION,
   STRAIGHTEN_CHAIN and FULL_TRANSFORM need explicit behaviour. STRAIGHTEN_CHAIN
   is the one the new `joint_limits` could plausibly serve.
5. **The constraint extension point is a different shape.** GodotIK's
   extensibility is a per-bone virtual `apply(parent, bone, child, direction)`
   returning three positions, in skeleton-local space. The pole vector and angle
   limits here are post-solve projections on world-space joints. Any project
   using custom GodotIK constraints loses them on a swap unless a
   `FabrikConstraint` with the same signature is provided.
6. **Ancestor propagation and rest pose.** GodotIK keeps `initial_transforms`
   and propagates from a chain's ancestor (`closest_parent_in_chain`,
   `pivot_child_in_ancestor`). `pose_skeleton()` here does not compensate rest
   orientation (a documented limitation) and `FabrikRig3D`'s dependency order is
   a different mechanism with a different failure mode. Chains that overlap will
   diverge.
7. **Space conversion is mandatory with (1).** GodotIK works in skeleton-local
   space; `FabrikChain3D` works in whatever space the caller feeds it. Any
   skeleton with a non-identity transform diverges unless the modifier converts
   both ways, including the pole target.
8. **Iteration defaults differ.** This adapter defaults to 64 iterations and
   tolerance 1e-5; GodotIK defaults to `iteration_count = 8`. A hotswap can
   therefore never mean bit-identical poses. It has to mean *same scene,
   comparable pose*, demonstrated rather than asserted.
9. **Tooling.** A scene converter plus a dual-run parity test (both solvers on
   one skeleton, per-bone delta reported) is what would turn "hotswappable"
   from a claim into a measurement.

## Recommendation

Build 1 + 2 + 3 first: together they are the minimum for a scene to be able to
point at this adapter instead of GodotIK. Then 4 and 6 for behavioural parity.
Item 5 only if the consuming project actually uses custom constraints. Item 7
comes with item 1. The cheapest honest milestone is 1 + 2 + 3 plus a demo scene
that runs both solvers on one skeleton and prints the largest per-bone delta.

Keep all of it in this repository. Deciding whether the game takes a
backend-agnostic façade (and which one) is a project-level decision, and the
alternative to a hotswap — a straight replacement behind one interface — is worth
weighing explicitly, because a hotswap implies a scene that can hold both
implementations at once.
