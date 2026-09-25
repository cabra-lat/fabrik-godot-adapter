# Adapter API

`FabrikChain3D` is registered through the GDExtension class DB and is usable
from GDScript:

```gdscript
var chain = FabrikChain3D.new()
chain.joints = PackedVector3Array([Vector3.ZERO, Vector3.RIGHT, Vector3.RIGHT * 2.0])
chain.segment_lengths = PackedFloat32Array([1.0, 1.0])
chain.target = Vector3(1.0, 1.0, 0.0)
var status = chain.solve()
if status == 0:
    print(chain.last_residual)
```

The adapter copies packed arrays into a flat C buffer before calling the core;
Fortran descriptors and derived types never enter Godot. A failed solve still
returns a deterministic chain when the core can produce one, so callers should
always check `status` before consuming `last_residual`.

## Rotations and skeletons

`get_joint_rotations() -> Array[Quaternion]` derives one orientation per joint
from the solved positions. The FABRIK core is a position solver, so the
orientation step lives in the adapter. The first frame is seeded with a
reference axis that is not parallel to the root bone, and every following frame
is parallel transported along the chain, so bending the chain does not flip a
bone. Each quaternion has `+Y` along the segment it spans, matching the
`Skeleton3D` bone convention.

`pose_skeleton(skeleton: Skeleton3D, bone_names: PackedStringArray) -> int`
writes those rotations onto a skeleton. It returns the number of bones posed,
or a negative code:

| return | meaning |
| --- | --- |
| `>= 0` | number of bones posed |
| `-1` | the skeleton argument was null |
| `-2` | `bone_names` and the joint count disagree |
| `-3` | a bone name does not exist in the skeleton |

Rest-pose compensation is not handled: poses are written as-is, so a rig whose
bones are not authored along local `+Y` must fold the rest orientation in
itself.

## Pole targets

FABRIK fixes the chain's reach and segment lengths, but those constraints leave
the bend plane underdetermined. Set `pole_target` to a world-space point on the
desired side of the chain to choose that plane:

```gdscript
chain.pole_target = Vector3(0.0, 1.0, 1.0)
```

The adapter rotates the intermediate joints around the root-to-tip axis after
the position solve. Because that is a rigid rotation about a fixed axis, the
root and tip stay put and every declared segment length is preserved. A zero
pole target disables the constraint.

## Joint angle limits

FABRIK constrains segment lengths only, so on its own it will straighten a knee
backwards or fold an elbow past its flexion limit. Limits are a post-solve
projection, given per joint as the **interior** angle in degrees — `180` is
straight, `0` is folded back on itself:

```gdscript
chain.joint_limits = PackedVector2Array([
    Vector2(0, 0),      # joint 0: the root has no interior angle
    Vector2(0, 60),     # this joint may fold to 60 degrees, never straighter
    Vector2(0, 0),      # joint 2: the tip, likewise
])
```

- `get_joint_angles()` returns the measured interior angle per joint in degrees
  (the two ends report `0`).
- A limit is enforced by rotating the sub-chain **below** the joint. That keeps
  an anchored root exactly where it is and preserves every segment length,
  because the sub-chain moves rigidly; the tip is what gives way. A limit
  therefore makes the target genuinely unreachable, and `get_last_residual()`
  reports the shortfall instead of claiming a clean solve.
- This is deliberately **not** the mechanism Aristidou, Chrysanthou & Lasenby
  (2015) describe. That paper enforces joint restrictions by re-positioning the
  *target* into the allowable bounds at every iteration, so the result stays a
  true FABRIK solution of a clamped target. The post-solve projection used here
  is far simpler and keeps the root and lengths exact, but the resulting pose is
  not a FABRIK solution of any single target. A target-clamping variant is the
  natural next step if a rig needs the "still a real FABRIK solution" property.
- Fixing one joint can move the next, so the projection runs up to
  `set_limit_iterations()` passes (default 4).
- `get_limit_projection_count()` counts the projections of the last solve;
  `get_limit_violation_count()` counts joints still outside their range
  afterwards. `0` violations is the healthy case — a non-zero value means the
  limits and the pose could not both hold, which is reported rather than
  hidden.
- A shorter `joint_limits` array leaves the remaining joints unlimited, and a
  `Vector2(0, 180)` entry is unlimited too.
- Limits act on positions, so they are applied before the rotation-space
  smoothing pass and are not themselves eased.

## Solving a whole rig in order (`FabrikRig3D`)

A body is not a set of independent chains: a hand or a prop target usually lives
somewhere on an arm that should already be solved this frame. `FabrikRig3D`
owns an ordered list of `FabrikChain3D` objects and solves them in a
deterministic order derived from the declaration order plus explicit
dependencies.

```gdscript
var rig := FabrikRig3D.new()
rig.add_chain(thigh)
rig.add_chain(shin)          # declared before its parent on purpose
rig.add_dependency(shin, thigh)   # "solve thigh before shin"

rig.set_target_provider(shin, func(_rig: FabrikRig3D, _index: int) -> Vector3:
	return rig.get_chains()[0].joints[-1] + Vector3(0, 0, 0.1))

if not rig.solve_all():
	push_error(rig.get_last_error())
```

- `add_chain` ignores nulls and duplicates: solving a chain twice per frame
  would ease it twice and quietly make its smoothing frame-rate dependent.
- `get_solve_order()` returns the indices actually solved, in that order;
  `get_chains()` is the declaration order they index into.
- `move_chain` reorders the declaration, which is the tie-break when no
  dependency decides.
- `set_target_provider(chain, callable)` is called as `callable(rig, index)`
  immediately before that chain's solve, so a target can be derived from a chain
  solved earlier in the same frame. It must return a `Vector3`; anything else
  leaves the chain's current target in place and is counted in
  `get_last_provider_failures()` rather than being silently ignored.
- `get_last_statuses()` holds one status per chain, in solve order.
- `solve_all()` returns `false` — and solves **nothing** — when the dependencies
  contain a cycle or a chain has been freed. A half-solved rig looks posed but is
  not, so the whole frame is refused and `get_last_error()` says why.

Dependencies are keyed by instance id, so reordering or removing a chain cannot
leave a stale index behind; removing a chain also drops its edges.

What this is **not**: a closed-loop solver. Chains are still solved one at a
time and no constraint is projected across chains — a cycle is reported, not
relaxed. Collision and scene-tree ownership are out of scope too.

## References

All four were read in full text, and are cited for what they say:

- R. Aristidou, N. Chr. Chrysanthou, J. Lasenby, *Extending FABRIK with model
  constraints*, Computer Animation and Virtual Worlds 27(1), 2015, pp. 35-57,
  <https://doi.org/10.1002/cav.1630> — §5 covers anthropometric and robotic
  joint models, and states that restrictions are enforced by re-positioning the
  target within the allowable bounds at each iteration; it also treats leaf
  joints, closed loops, fixed inter-joint distance, and unreachable targets.
- A. Aristidou, J. Lasenby, *FABRIK: A fast, iterative solver for the Inverse
  Kinematics problem*, Graphical Models 73(5), 2011, pp. 243-260,
  <https://doi.org/10.1016/j.gmod.2011.05.003> — the algorithm, its pseudo-code
  and its unreachable-target case.
- M. C. Santos et al., *FABRIK-R: An Extension Developed Based on FABRIK for
  Robotics Manipulators*, IEEE Access, 2021,
  <https://doi.org/10.1109/ACCESS.2021.3070693> — FABRIK on 1-DOF joint chains.
- Z. Xu et al., *A Combined Inverse Kinematics Algorithm Using FABRIK with
  Optimization*, arXiv:2209.02532, 2022,
  <https://arxiv.org/abs/2209.02532> — reports FABRIK's unstable convergence
  under high Cartesian error constraints, which is why the core's iteration
  budget and tolerance are explicit and its status codes are checked.

The core repository carries the same list, and the wider provenance record:
<https://github.com/cabra-lat/fabrik-fortran/blob/13253a77b78b515b89d2531dd93aa6934e80b523/docs/ALGORITHM.md>
(commit pinned by `.github/workflows/ci.yml` as `FABRIK_CORE_REF`).

## Smoothing

`smoothing` is in `[0, 1]`. Each solve interpolates the result towards the
previous pose by `1 - smoothing`, so `0` is the raw solve and `1` never moves.
This is what keeps a per-frame solve from snapping: the chain eases towards the
target instead of teleporting to it.

## Signals

`solve_finished(status: int, residual: float)` is emitted after a successful
(`FABRIK_OK`) solve.
