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
