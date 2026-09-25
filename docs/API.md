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

## Smoothing

`smoothing` is in `[0, 1]`. Each solve interpolates the result towards the
previous pose by `1 - smoothing`, so `0` is the raw solve and `1` never moves.
This is what keeps a per-frame solve from snapping: the chain eases towards the
target instead of teleporting to it.

## Signals

`solve_finished(status: int, residual: float)` is emitted after a successful
(`FABRIK_OK`) solve.
