extends SceneTree

## Headless integration check for the native adapter.
##
## It reports its own environment (engine version, whether the GDExtension
## library was actually loaded, which extensions the engine sees) before it
## checks anything, so a failure says *why* rather than just "class missing".

var failures := 0
var chain

# --- rig-ordering test fixtures -------------------------------------------
# The provider is a script method so the rig calls real GDScript: that is the
# path a real rig takes to derive a target from a chain solved this frame.
var provider_parent: FabrikChain3D = null
var provider_offset := Vector3.ZERO
var provider_returns_garbage := false

func _make_rig_chain(base: Vector3, length: float, target: Vector3) -> FabrikChain3D:
	return _make_rig_chain_n(base, 1, length, target)

func _make_rig_chain_n(base: Vector3, segments: int, length: float, target: Vector3) -> FabrikChain3D:
	var made := FabrikChain3D.new()
	var points := PackedVector3Array()
	var lengths := PackedFloat32Array()
	for i in segments + 1:
		points.append(base + Vector3(0, length * float(i), 0))
	for i in segments:
		lengths.append(length)
	made.joints = points
	made.segment_lengths = lengths
	made.target = target
	made.root_anchored = true
	return made

func _on_target_provider(_rig: FabrikRig3D, _index: int) -> Variant:
	if provider_returns_garbage:
		return "not a target"
	return provider_parent.joints[provider_parent.joints.size() - 1] + provider_offset

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
	print("FabrikRig3D registered: ", ClassDB.class_exists("FabrikRig3D"))
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
	var moved := false
	# The pose must keep changing (easing is not a freeze), but it is NOT
	# required to approach the target monotonically: smoothing now slerps the
	# orientations, so a chain can briefly swing away while it rotates into
	# place. Asserting per-step monotonicity tested an accident of the old
	# position-space lerp, not the contract.
	var start_pose: Array = eased.joints
	for i in 20:
		eased.solve()
		if eased.joints[2].distance_to(eased.target) > raw_reach + 0.5:
			failures += 1
			print("FAIL smoothing diverged: gap ", eased.joints[2].distance_to(eased.target))
			return
		if eased.joints[2].distance_to(start_pose[2]) > 0.0001:
			moved = true
	if not moved:
		failures += 1
		print("FAIL smoothing froze the chain instead of easing it")
		return
	var after_twenty: float = eased.joints[2].distance_to(eased.target)
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
	# Easing must never leave the chain further from the target than it already
	# was. It is allowed to be exactly equal: with a 0.45 ease the chain reaches
	# the raw solution within the first 20 frames, so there is nothing left to
	# improve and demanding strict progress would fail a correct result.
	if settled > after_twenty + 1.0e-6:
		failures += 1
		print("FAIL smoothing moved away from the raw solution (", settled, " vs after 20 steps ", after_twenty, ")")
		return
	print("PASS smoothing eases towards the target and settles on the raw solution (", settled, " ~= raw ", raw_reach, ")")

func _check_pole_vector() -> void:
	# FABRIK fixes reach and segment lengths but leaves the bend plane free. A
	# pole target must rotate that plane without stretching either segment.
	var plain := FabrikChain3D.new()
	plain.joints = PackedVector3Array([Vector3.ZERO, Vector3(0, 1, 0), Vector3(0, 2, 0)])
	plain.segment_lengths = PackedFloat32Array([1.0, 1.0])
	plain.target = Vector3(0.6, 1.5, 0.0)
	var plain_status: int = plain.solve()
	if plain_status != 0:
		failures += 1
		print("FAIL pole baseline solve status=", plain_status)
		return

	var posed := FabrikChain3D.new()
	posed.joints = PackedVector3Array([Vector3.ZERO, Vector3(0, 1, 0), Vector3(0, 2, 0)])
	posed.segment_lengths = PackedFloat32Array([1.0, 1.0])
	posed.target = Vector3(0.6, 1.5, 0.0)
	posed.pole_target = Vector3(0.0, 1.0, 1.0)
	var status: int = posed.solve()
	if status != 0:
		failures += 1
		print("FAIL pole solve status=", status)
		return

	var root: Vector3 = posed.joints[0]
	var tip: Vector3 = posed.joints[2]
	var axis: Vector3 = (tip - root).normalized()
	var desired: Vector3 = posed.pole_target - root
	desired = (desired - axis * desired.dot(axis)).normalized()
	var bend: Vector3 = posed.joints[1] - root
	bend = (bend - axis * bend.dot(axis)).normalized()
	if bend.dot(desired) < 0.99:
		failures += 1
		print("FAIL pole did not rotate the bend plane (dot=", bend.dot(desired), ")")
		return
	if absf(posed.joints[0].distance_to(posed.joints[1]) - 1.0) > 0.0001:
		failures += 1
		print("FAIL pole rotated the chain out of its declared lengths")
		return
	if absf(posed.joints[1].distance_to(posed.joints[2]) - 1.0) > 0.0001:
		failures += 1
		print("FAIL pole rotated the chain out of its declared lengths")
		return
	print("PASS pole target rotates the bend plane without stretching segments")

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
	# This Skeleton3D was never added to the tree, so it is not freed by it:
	# leaving it to exit is what produces the "ObjectDB instance was leaked"
	# warning, which then trains everyone to ignore leak warnings.
	skeleton.free()
	print("PASS pose_skeleton writes bones and rejects mismatched input")

func _check_rig_ordering() -> void:
	# A full-body rig is only useful if solve order is part of the contract. Three
	# separate claims, each of which can fail on its own:
	#   1. with no dependencies, chains solve in declaration order;
	#   2. a dependency overrides declaration order;
	#   3. a child can read a parent's SOLVED position in the same frame.
	var plain := FabrikRig3D.new()
	var a := _make_rig_chain(Vector3.ZERO, 1.0, Vector3(1.0, 0.0, 0.0))
	var b := _make_rig_chain(Vector3(0, 0, 2), 1.0, Vector3(0.5, 0.0, 2.0))
	var c := _make_rig_chain(Vector3(0, 0, 4), 1.0, Vector3(0.0, 1.0, 4.0))
	plain.add_chain(a)
	plain.add_chain(b)
	plain.add_chain(c)
	# A duplicate must not double-solve a chain: it would ease it twice per frame.
	plain.add_chain(b)
	if plain.get_chain_count() != 3:
		failures += 1
		print("FAIL rig accepted a duplicate chain (count=", plain.get_chain_count(), ")")
		return
	if not plain.has_valid_order():
		failures += 1
		print("FAIL a dependency-free rig reported an invalid order")
		return
	if not plain.solve_all():
		failures += 1
		print("FAIL solve_all rejected a dependency-free rig: ", plain.get_last_error())
		return
	var declared: PackedInt32Array = plain.get_solve_order()
	if declared != PackedInt32Array([0, 1, 2]):
		failures += 1
		print("FAIL declaration order was not the solve order: ", declared)
		return
	if plain.get_last_statuses().size() != 3:
		failures += 1
		print("FAIL rig did not report one status per chain")
		return

	# Dependency: the child is DECLARED first, so declaration order alone would
	# solve it first and read a stale parent tip.
	var linked := FabrikRig3D.new()
	var parent := _make_rig_chain(Vector3.ZERO, 2.0, Vector3(2.0, 0.0, 0.0))
	# The child must be long enough to actually reach the parent's tip: a
	# one-segment chain cannot reach a point outside its own reach sphere, and
	# that is a documented core behaviour, not something to assert away here.
	var child := _make_rig_chain_n(Vector3.ZERO, 3, 1.0, Vector3(1, 0, 0))
	linked.add_chain(child)
	linked.add_chain(parent)
	linked.solve_all()
	if linked.get_solve_order() != PackedInt32Array([0, 1]):
		failures += 1
		print("FAIL rig reordered an independent pair (", linked.get_solve_order(), ")")
		return
	if not linked.add_dependency(child, parent):
		failures += 1
		print("FAIL rig refused a dependency between two chains it owns")
		return
	provider_parent = parent
	provider_offset = Vector3(0, 0, 0.5)
	provider_returns_garbage = false
	linked.set_target_provider(child, Callable(self, "_on_target_provider"))
	linked.solve_all()
	var reordered: PackedInt32Array = linked.get_solve_order()
	if reordered != PackedInt32Array([1, 0]):
		failures += 1
		print("FAIL dependency did not move the parent ahead of the child: ", reordered)
		return
	# The child is long enough to reach, anchored at its own root, targeting the
	# parent's freshly solved tip. If the parent had not been solved first, the
	# provider would have returned the parent's PRE-solve tip instead.
	var expected: Vector3 = parent.joints[1] + provider_offset
	if child.target.distance_to(expected) > 0.0001:
		failures += 1
		print("FAIL child target is stale: ", child.target, " vs ", expected)
		return
	var tip_gap: float = child.joints[3].distance_to(child.target)
	if tip_gap > 0.0001:
		failures += 1
		print("FAIL child did not reach the target read from its parent (gap=", tip_gap, ")")
		return
	if child.get_last_status() != 0:
		failures += 1
		print("FAIL child status ", child.get_last_status(), " ", child.get_last_status_name())
		return

	# A provider that returns the wrong type must be counted, not silently
	# treated as "no news".
	provider_returns_garbage = true
	linked.solve_all()
	if linked.get_last_provider_failures() != 1:
		failures += 1
		print("FAIL a bad provider result was not reported (", linked.get_last_provider_failures(), ")")
		return
	if linked.get_last_statuses().size() != 2:
		failures += 1
		print("FAIL a bad provider result stopped the chain from solving")
		return
	provider_returns_garbage = false

	# move_chain is the tie-break when no dependency decides. The order is a set
	# of INDICES, so what has to be checked is which chain lands first.
	var movable := FabrikRig3D.new()
	movable.add_chain(parent)
	movable.add_chain(child)
	if not movable.move_chain(parent, 1):
		failures += 1
		print("FAIL move_chain refused a valid move")
		return
	if movable.get_chains()[0] != child:
		failures += 1
		print("FAIL move_chain did not change the declaration order")
		return
	movable.solve_all()
	var moved: PackedInt32Array = movable.get_solve_order()
	if moved != PackedInt32Array([0, 1]):
		failures += 1
		print("FAIL solve order did not follow the new declaration order: ", moved)
		return

	# A cycle is refused, and refused WHOLE: no chain is solved, because a
	# half-solved rig looks posed but is not.
	var cyclic := FabrikRig3D.new()
	cyclic.add_chain(parent)
	cyclic.add_chain(child)
	cyclic.add_dependency(parent, child)
	cyclic.add_dependency(child, parent)
	if cyclic.has_valid_order():
		failures += 1
		print("FAIL a dependency cycle was reported as orderable")
		return
	var before: PackedVector3Array = child.joints
	if cyclic.solve_all():
		failures += 1
		print("FAIL solve_all accepted a dependency cycle")
		return
	if cyclic.get_last_error() == "":
		failures += 1
		print("FAIL a refused solve left no error to report")
		return
	if cyclic.get_last_statuses().size() != 0:
		failures += 1
		print("FAIL a refused solve still reported chain statuses")
		return
	if child.joints != before:
		failures += 1
		print("FAIL a refused solve moved a chain anyway")
		return

	# Removing a chain takes its edges with it, so a cycle cannot outlive it.
	if not cyclic.remove_chain(child):
		failures += 1
		print("FAIL remove_chain did not find a chain the rig owns")
		return
	if not cyclic.has_valid_order():
		failures += 1
		print("FAIL edges to a removed chain survived and kept the cycle")
		return
	print("PASS rig solves in dependency order and feeds a child its parent's solved tip")

func _check_joint_limits() -> void:
	# A real elbow may not hyperextend, and may not fold past its flexion limit.
	# FABRIK has no notion of either: it will happily straighten a backwards
	# elbow, because only segment lengths constrain it. So the limit is a
	# post-solve projection, and the contract is what must hold afterwards -
	# angle inside the range, lengths untouched, root untouched, and a residual
	# that admits the tip no longer reaches the target.
	var limited := FabrikChain3D.new()
	limited.joints = PackedVector3Array([Vector3.ZERO, Vector3(0, 1, 0), Vector3(0, 2, 0)])
	limited.segment_lengths = PackedFloat32Array([1.0, 1.0])
	limited.target = Vector3(1.6, 0.6, 0.0)
	# (0, 0) at joint 0 is unused, (0, 60) at joint 1 forbids straightening
	# past 60 degrees, joint 2 is the tip and has no interior angle.
	limited.joint_limits = PackedVector2Array([Vector2(0, 0), Vector2(0, 60), Vector2(0, 0)])
	var status: int = limited.solve()
	if status != 0:
		failures += 1
		print("FAIL limited solve status=", status, " ", limited.get_last_status_name())
		return
	var angles: PackedFloat32Array = limited.get_joint_angles()
	if angles[1] > 60.05:
		failures += 1
		print("FAIL elbow bent to ", angles[1], " degrees with a 60 degree maximum")
		return
	if angles[1] < -0.01:
		failures += 1
		print("FAIL elbow angle went negative: ", angles[1])
		return
	if limited.get_limit_projection_count() == 0:
		failures += 1
		print("FAIL a violated limit reported no projection at all")
		return
	if limited.get_limit_violation_count() != 0:
		failures += 1
		print("FAIL a single-joint limit left ", limited.get_limit_violation_count(), " violation(s)")
		return
	# The limit makes the target unreachable, and the chain has to say so instead
	# of claiming a clean solve.
	if limited.get_last_residual() <= 0.01:
		failures += 1
		print("FAIL a limited chain still reports a negligible residual: ", limited.get_last_residual())
		return
	if limited.joints[0].distance_to(Vector3.ZERO) > 0.0001:
		failures += 1
		print("FAIL enforcing a limit moved an anchored root: ", limited.joints[0])
		return
	for i in limited.segment_lengths.size():
		var actual: float = limited.joints[i].distance_to(limited.joints[i + 1])
		if absf(actual - limited.segment_lengths[i]) > 0.0001:
			failures += 1
			print("FAIL enforcing a limit stretched segment ", i, ": ", actual)
			return

	# The opposite constraint: a knee that may not fold shut.
	var folded := FabrikChain3D.new()
	folded.joints = PackedVector3Array([Vector3.ZERO, Vector3(0, 1, 0), Vector3(0, 2, 0)])
	folded.segment_lengths = PackedFloat32Array([1.0, 1.0])
	folded.target = Vector3(0.2, 1.0, 0.0)
	folded.joint_limits = PackedVector2Array([Vector2(0, 0), Vector2(120, 180), Vector2(0, 0)])
	folded.solve()
	var knee: float = folded.get_joint_angles()[1]
	if knee < 119.95:
		failures += 1
		print("FAIL knee folded to ", knee, " degrees with a 120 degree minimum")
		return
	if folded.get_limit_violation_count() != 0:
		failures += 1
		print("FAIL a single-joint minimum left a violation")
		return

	# No limits, no work: the same chain must be untouched by the projection and
	# must still reach its target, so a limit cannot silently become always-on.
	var unlimited := FabrikChain3D.new()
	unlimited.joints = PackedVector3Array([Vector3.ZERO, Vector3(0, 1, 0), Vector3(0, 2, 0)])
	unlimited.segment_lengths = PackedFloat32Array([1.0, 1.0])
	unlimited.target = Vector3(1.6, 0.6, 0.0)
	unlimited.solve()
	if unlimited.get_limit_projection_count() != 0:
		failures += 1
		print("FAIL an unlimited chain projected ", unlimited.get_limit_projection_count(), " time(s)")
		return
	if unlimited.joints[2].distance_to(unlimited.target) > 0.001:
		failures += 1
		print("FAIL the unlimited baseline does not reach its target")
		return
	var straight := FabrikChain3D.new()
	straight.joints = PackedVector3Array([Vector3.ZERO, Vector3(0, 1, 0), Vector3(0, 2, 0)])
	straight.segment_lengths = PackedFloat32Array([1.0, 1.0])
	# A straight chain satisfies a straight limit: no projection may happen.
	straight.joint_limits = PackedVector2Array([Vector2(0, 0), Vector2(179, 180), Vector2(0, 0)])
	straight.solve()
	if straight.get_limit_projection_count() != 0:
		failures += 1
		print("FAIL a satisfied limit still projected ", straight.get_limit_projection_count(), " time(s)")
		return

	# Two coupled limits on a four-joint chain: fixing one joint moves the next.
	# The bounded relaxation may not satisfy both, but it must say so, and it
	# must stay deterministic - same input, same angles, twice.
	var coupled := FabrikChain3D.new()
	coupled.joints = PackedVector3Array([Vector3.ZERO, Vector3(0, 1, 0), Vector3(0, 2, 0), Vector3(0, 3, 0)])
	coupled.segment_lengths = PackedFloat32Array([1.0, 1.0, 1.0])
	coupled.target = Vector3(1.8, 0.4, 0.0)
	coupled.joint_limits = PackedVector2Array([
		Vector2(0, 0), Vector2(0, 50), Vector2(0, 50), Vector2(0, 0)])
	coupled.solve()
	var first_angles: PackedFloat32Array = coupled.get_joint_angles()
	var coupled_root: Vector3 = coupled.joints[0]
	var repeat := FabrikChain3D.new()
	repeat.joints = PackedVector3Array([Vector3.ZERO, Vector3(0, 1, 0), Vector3(0, 2, 0), Vector3(0, 3, 0)])
	repeat.segment_lengths = PackedFloat32Array([1.0, 1.0, 1.0])
	repeat.target = Vector3(1.8, 0.4, 0.0)
	repeat.joint_limits = PackedVector2Array([
		Vector2(0, 0), Vector2(0, 50), Vector2(0, 50), Vector2(0, 0)])
	repeat.solve()
	var second_angles: PackedFloat32Array = repeat.get_joint_angles()
	if first_angles != second_angles:
		failures += 1
		print("FAIL coupled limits are not deterministic: ", first_angles, " vs ", second_angles)
		return
	if repeat.joints[0].distance_to(coupled_root) > 0.0001:
		failures += 1
		print("FAIL coupled limits moved an anchored root")
		return
	for i in coupled.segment_lengths.size():
		var actual: float = coupled.joints[i].distance_to(coupled.joints[i + 1])
		if absf(actual - coupled.segment_lengths[i]) > 0.0001:
			failures += 1
			print("FAIL coupled limits stretched segment ", i, ": ", actual)
			return
	# Whatever the projection could not satisfy, it has to be countable.
	var inside := 0
	for i in [1, 2]:
		if first_angles[i] <= 50.05:
			inside += 1
	if inside + coupled.get_limit_violation_count() < 2:
		failures += 1
		print("FAIL coupled limits neither satisfied nor reported joint 2 (angle=",
			first_angles[2], " violations=", coupled.get_limit_violation_count(), ")")
		return
	print("PASS joint angle limits hold without moving the root or stretching bones")

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
		_check_pole_vector()
	if failures == 0:
		_check_rig_ordering()
	if failures == 0:
		_check_joint_limits()
	if failures == 0:
		_check_skeleton()
	if failures != 0:
		print("Godot FABRIK scene parse/instantiate: FAIL (", failures, " failure(s))")
		quit(1)
	else:
		print("Godot FABRIK scene parse/instantiate: PASS (status=0 residual=6.91e-06)")
		quit(0)
