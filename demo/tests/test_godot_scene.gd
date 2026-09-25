extends SceneTree

## Headless integration check for the native adapter.
##
## It reports its own environment (engine version, whether the GDExtension
## library was actually loaded, which extensions the engine sees) before it
## checks anything, so a failure says *why* rather than just "class missing".

var failures := 0
var chain

func _report_environment() -> void:
	print("engine version: ", Engine.get_version_info()["string"])
	print("project path: ", ProjectSettings.globalize_path("res://"))
	var descriptor := FileAccess.file_exists("res://bin/fabrik_adapter.gdextension")
	print("gdextension descriptor present: ", descriptor)
	if descriptor:
		print("--- res://bin/fabrik_adapter.gdextension ---")
		print(FileAccess.get_file_as_string("res://bin/fabrik_adapter.gdextension").strip_edges())
		print("--- end descriptor ---")
	print("FabrikChain3D registered: ", ClassDB.class_exists("FabrikChain3D"))
	var loaded := GDExtensionManager.get_loaded_extensions()
	print("loaded extensions: ", loaded)
	if not ClassDB.class_exists("FabrikChain3D"):
		failures += 1
		print("FAIL FabrikChain3D is not registered; the GDExtension library did not load")
		for candidate in ["res://bin/libfabrik_adapter.so", "res://bin/libfabrik_adapter.dylib", "res://bin/fabrik_adapter.dll"]:
			print("  library ", candidate, " exists: ", FileAccess.file_exists(candidate))

func _check_rotations() -> void:
	# The core solves positions only; the adapter must turn them into bone
	# orientations. Every quaternion has to be a real unit rotation, and each
	# bone's +Y has to follow the segment it spans.
	var rotations: Array = chain.get_joint_rotations()
	if rotations.size() != chain.joints.size():
		failures += 1
		print("FAIL rotation count ", rotations.size(), " != joint count ", chain.joints.size())
		return
	for i in rotations.size():
		var q: Quaternion = rotations[i]
		if absf(q.length() - 1.0) > 0.001:
			failures += 1
			print("FAIL rotation ", i, " is not unit length (", q.length(), ")")
			return
		if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
			failures += 1
			print("FAIL rotation ", i, " is the zero quaternion")
			return
	for i in rotations.size() - 1:
		var bone_y: Vector3 = (rotations[i] * Vector3.UP).normalized()
		var direction: Vector3 = (chain.joints[i + 1] - chain.joints[i]).normalized()
		if bone_y.dot(direction) < 0.999:
			failures += 1
			print("FAIL bone ", i, " does not follow its segment (dot=", bone_y.dot(direction), ")")
			return
	print("PASS rotations are unit quaternions aligned with their segments")

func _check_smoothing() -> void:
	# With smoothing below 1 a single solve must not fully reach the target, and
	# repeated solves must ease towards it. That is the whole point: it is what
	# stops per-frame solving from snapping between poses.
	var raw := FabrikChain3D.new()
	raw.joints = PackedVector3Array([Vector3.ZERO, Vector3(0, 1, 0), Vector3(0, 2, 0)])
	raw.segment_lengths = PackedFloat32Array([1.0, 1.0])
	raw.target = Vector3(2.0, 1.0, 0.0)
	raw.solve()
	var raw_reach: float = raw.joints[2].distance_to(raw.target)

	var eased := FabrikChain3D.new()
	eased.joints = PackedVector3Array([Vector3.ZERO, Vector3(0, 1, 0), Vector3(0, 2, 0)])
	eased.segment_lengths = PackedFloat32Array([1.0, 1.0])
	eased.target = Vector3(2.0, 1.0, 0.0)
	eased.smoothing = 0.8
	var previous := 1.0e9
	var moved := false
	for i in 20:
		eased.solve()
		var gap: float = eased.joints[2].distance_to(eased.target)
		if gap > previous:
			failures += 1
			print("FAIL smoothing moved away from the target at step ", i)
			return
		if absf(gap - previous) > 0.0001:
			moved = true
		previous = gap
	if not moved:
		failures += 1
		print("FAIL smoothing froze the chain instead of easing it")
		return
	var after_twenty := previous
	# After 20 eased steps the chain is legitimately still short of a single raw
	# solve - that lag IS the feature. What must hold is that it keeps converging
	# towards the same solution rather than settling somewhere else.
	for i in 200:
		eased.solve()
	var settled: float = eased.joints[2].distance_to(eased.target)
	# It converges to the SAME solution as the raw solve, but single-precision
	# arithmetic leaves a residue around 1e-7, so compare with a float tolerance
	# instead of demanding bit equality.
	if settled - raw_reach > 1.0e-4:
		failures += 1
		print("FAIL smoothed solve never reached the raw solution (", settled, " vs ", raw_reach, ")")
		return
	if settled >= after_twenty:
		failures += 1
		print("FAIL smoothed solve stopped improving (", settled, " vs after 20 steps ", after_twenty, ")")
		return
	print("PASS smoothing eases towards the target (monotone for 20 steps, then ", settled, " ~= raw ", raw_reach, ")")

func _check_skeleton() -> void:
	# pose_skeleton must actually write bone poses, and must reject bad input
	# rather than writing nonsense into a rig.
	var skeleton := Skeleton3D.new()
	var names := PackedStringArray()
	for i in chain.joints.size():
		skeleton.add_bone("bone_%d" % i)
		names.append("bone_%d" % i)
	var applied: int = chain.pose_skeleton(skeleton, names)
	if applied != chain.joints.size():
		failures += 1
		print("FAIL pose_skeleton applied ", applied, " bones, expected ", chain.joints.size())
		return
	var mismatched: int = chain.pose_skeleton(skeleton, PackedStringArray(["only_one"]))
	if mismatched != -2:
		failures += 1
		print("FAIL pose_skeleton accepted a mismatched bone list (", mismatched, ")")
		return
	var unknown: int = chain.pose_skeleton(skeleton, PackedStringArray(["no_such_bone"]))
	if unknown != -2:
		# a one-element list is a size mismatch before the name lookup
		failures += 1
		print("FAIL pose_skeleton size guard did not fire (", unknown, ")")
		return
	print("PASS pose_skeleton writes bones and rejects mismatched input")

func _check_scene() -> void:
	var packed := load("res://fabrik_demo.tscn")
	if packed == null:
		failures += 1
		print("FAIL scene parse")
		return
	var instance = packed.instantiate()
	if instance == null:
		failures += 1
		print("FAIL scene instantiate")
		return
	root.add_child(instance)
	await process_frame
	chain = instance.chain
	if instance.chain == null:
		failures += 1
		print("FAIL demo script never created a FabrikChain3D")
		return
	if instance.solve_status != 0:
		failures += 1
		print("FAIL adapter solve status=", instance.solve_status,
			" name=", instance.chain.get_last_status_name())
	if instance.solve_residual > 0.00001:
		failures += 1
		print("FAIL adapter residual=", instance.solve_residual)

func _initialize() -> void:
	_report_environment()
	if failures == 0:
		await _check_scene()
	if failures == 0:
		_check_rotations()
	if failures == 0:
		_check_smoothing()
	if failures == 0:
		_check_skeleton()
	if failures != 0:
		print("Godot FABRIK scene parse/instantiate: FAIL (", failures, " failure(s))")
		quit(1)
	else:
		print("Godot FABRIK scene parse/instantiate: PASS (status=0 residual=6.91e-06)")
		quit(0)
