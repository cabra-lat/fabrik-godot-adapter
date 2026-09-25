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

## Smoothing

`smoothing` is in `[0, 1]`. Each solve interpolates the result towards the
previous pose by `1 - smoothing`, so `0` is the raw solve and `1` never moves.
This is what keeps a per-frame solve from snapping: the chain eases towards the
target instead of teleporting to it.

## Signals

`solve_finished(status: int, residual: float)` is emitted after a successful
(`FABRIK_OK`) solve.
