extends SceneTree

## Headless assertions for the scene-tree layer: FabrikEffector +
## FabrikModifier3D, the part that makes a Skeleton3D solvable without a
## per-frame script.
##
## What is deliberately checked here, and not in the chain tests:
##   - the engine really drives the modifier (no solve_now() in that one test);
##   - influence is a measured blend, not a flag;
##   - results are correct for a skeleton that is not at the origin, which is
##     the space-conversion claim;
##   - the four transform modes differ from each other in the way they claim;
##   - a chain that cannot be built is reported, not silently skipped.

const CHAIN_LEN := 3
const TIP := 3
const CHAIN_ROOT := 1

var failures := 0

func _initialize() -> void:
	await process_frame

	_classes_are_registered()
	await _engine_drives_the_modifier()
	_influence_blends()
	_skeleton_transform_does_not_matter()
	_segment_lengths_survive()
	_transform_modes_differ()
	_unusable_chains_are_reported()
	_finish()

func _fail(message: String, detail: String = "") -> void:
	failures += 1
	if detail.is_empty():
		printerr("FAIL ", message)
	else:
		printerr("FAIL ", message, " ", detail)

func _finish() -> void:
	if failures == 0:
		print("Godot FABRIK modifier: PASS (0 failure(s))")
	else:
		printerr("RESULT: FAIL (", failures, " failure(s))")
	quit(1 if failures > 0 else 0)

# --- fixtures ---------------------------------------------------------------

## A straight four-bone column. The effector chain is bones 3, 2, 1 - two
## segments, so the chain can actually bend. A one-segment chain cannot: any
## goal off the segment is unreachable, which would make every assertion here a
## test of the unreachable path instead of the solve.
##
## Rest transforms are PARENT-relative, which was measured, not assumed: a rest
## of (0, i, 0) on every bone puts bone 3 at y=6, a rest of (0, 1, 0) puts it at
## y=3. The first version of this fixture used the wrong one and every assertion
## below failed on the numbers.
func _make_skeleton(skeleton_position: Vector3 = Vector3.ZERO) -> Skeleton3D:
	var skeleton := Skeleton3D.new()
	skeleton.position = skeleton_position
	root.add_child(skeleton)
	for i in 4:
		skeleton.add_bone(["root", "upper", "lower", "tip"][i])
		if i > 0:
			skeleton.set_bone_parent(i, i - 1)
			skeleton.set_bone_rest(i, Transform3D(Basis(), Vector3(0, 1, 0)))
	skeleton.reset_bone_poses()
	return skeleton

func _add_chain(skeleton: Skeleton3D, effector_bone: int, target: Vector3) -> Array:
	var modifier := FabrikModifier3D.new()
	modifier.name = "FabrikModifier3D"
	skeleton.add_child(modifier)
	var effector := FabrikEffector.new()
	effector.name = "Effector"
	modifier.add_child(effector)
	# The effector's own transform is the goal. With the modifier and skeleton
	# unrotated, its local position is already the skeleton-local goal.
	effector.position = target
	effector.bone_name = skeleton.get_bone_name(effector_bone)
	effector.chain_length = CHAIN_LEN
	skeleton.force_update_all_bone_transforms()
	return [modifier, effector]

func _leaf_position(skeleton: Skeleton3D, bone: int = TIP) -> Vector3:
	skeleton.force_update_all_bone_transforms()
	return skeleton.get_bone_global_pose(bone).origin

# --- tests ------------------------------------------------------------------

func _classes_are_registered() -> void:
	if not ClassDB.class_exists("FabrikEffector"):
		_fail("FabrikEffector is not registered")
	if not ClassDB.class_exists("FabrikModifier3D"):
		_fail("FabrikModifier3D is not registered")
		return
	if not ClassDB.is_parent_class("FabrikModifier3D", "SkeletonModifier3D"):
		_fail("FabrikModifier3D must derive from SkeletonModifier3D, got parent ",
			str(ClassDB.get_parent_class("FabrikModifier3D")))
	# The default iteration count is deliberately GodotIK's, not ours, so that a
	# hotswap does not change the pose by changing a default.
	var modifier := FabrikModifier3D.new()
	if modifier.iteration_count != 8:
		_fail("default iteration_count is " + str(modifier.iteration_count) + ", expected 8")
	modifier.free()

func _engine_drives_the_modifier() -> void:
	# The engine resets bone poses to the base pose, runs the modifiers, and then
	# rebuilds the pose cache - so a modifier's set_bone_pose() is not readable
	# through get_bone_pose() after the update returns. Measured, not assumed: a
	# write from inside the callback was invisible to get_bone_pose(), while a
	# write made from outside it stuck. So the claim under test is "the engine
	# calls the modifier, and the solve it produced is correct", read from the
	# modifier's own report rather than from the bones.
	var skeleton := _make_skeleton()
	var made := _add_chain(skeleton, TIP, Vector3(1.5, 0.5, 0.0))
	var modifier: FabrikModifier3D = made[0]
	var start := _leaf_position(skeleton)
	# Deliberately no solve_now(): this is the only test that proves the engine
	# calls the modifier, which is the whole reason for deriving from
	# SkeletonModifier3D.
	await process_frame
	await process_frame
	if modifier.get_last_chain_count() != 1:
		_fail("engine-driven solve reported " + str(modifier.get_last_chain_count())
			+ " chains, expected 1 (error: " + modifier.get_last_error() + ")")
		skeleton.queue_free()
		return
	var bones := modifier.get_last_bones()
	var solved := modifier.get_last_solved_positions()
	if bones.size() != CHAIN_LEN or solved.size() != CHAIN_LEN:
		_fail("engine-driven solve reported " + str(bones.size()) + " bones, expected "
			+ str(CHAIN_LEN))
		skeleton.queue_free()
		return
	# The chain is reported root-first, so the last position is the tip.
	var tip := solved[solved.size() - 1]
	if not modifier.get_last_statuses().has(0):
		_fail("engine-driven solve did not report OK: " + str(modifier.get_last_statuses())
			+ " residual " + str(modifier.get_last_max_residual()))
	elif tip.distance_to(Vector3(1.5, 0.5, 0.0)) > 0.05:
		_fail("engine-driven solve left the tip at " + str(tip) + ", expected near (1.5, 0.5, 0); statuses "
			+ str(modifier.get_last_statuses()) + " residual " + str(modifier.get_last_max_residual()))
	else:
		print("PASS engine drives the modifier: chain " + str(bones) + " solved from "
			+ str(start) + " to tip " + str(tip))
	skeleton.queue_free()

func _influence_blends() -> void:
	var skeleton := _make_skeleton()
	var made := _add_chain(skeleton, TIP, Vector3(1.5, 1.0, 0.0))
	var modifier: FabrikModifier3D = made[0]
	var effector: FabrikEffector = made[1]
	var rest := _leaf_position(skeleton)
	var goal := Vector3(1.5, 1.0, 0.0)

	# influence 0 must not move the bone at all, however long we wait.
	effector.influence = 0.0
	skeleton.reset_bone_poses()
	skeleton.force_update_all_bone_transforms()
	for i in 5:
		modifier.solve_now()
	var untouched := _leaf_position(skeleton)
	if not untouched.is_equal_approx(rest):
		_fail("influence 0 moved the bone: " + str(untouched) + " != " + str(rest))
	elif modifier.get_last_chain_count() != 0:
		_fail("influence 0 still solved a chain")

	# Half influence lands halfway between where the leaf is and the goal.
	skeleton.reset_bone_poses()
	skeleton.force_update_all_bone_transforms()
	effector.influence = 0.5
	modifier.solve_now()
	var half := _leaf_position(skeleton)
	var expected := rest.lerp(goal, 0.5)
	if half.distance_to(expected) > 0.02:
		_fail("influence 0.5 landed at " + str(half) + ", expected " + str(expected))

	# Full influence reaches the goal.
	skeleton.reset_bone_poses()
	skeleton.force_update_all_bone_transforms()
	effector.influence = 1.0
	modifier.solve_now()
	var full := _leaf_position(skeleton)
	if full.distance_to(goal) > 0.01:
		_fail("influence 1.0 landed at " + str(full) + ", expected " + str(goal))
	else:
		print("PASS influence blends 0 / 0.5 / 1 measurably")
	skeleton.queue_free()

func _skeleton_transform_does_not_matter() -> void:
	# The same skeleton at the origin and translated must solve to the same
	# skeleton-local pose. This is the space-conversion claim, so it is measured
	# rather than asserted.
	#
	# The subtlety that makes this test worth having: get_bone_global_pose()
	# reports SKELETON-LOCAL coordinates, not world. With the skeleton at
	# (10,20,30) a bone at skeleton-local (0,1,0) still reports (0,1,0). So the
	# skeleton's transform has to be applied by hand to get a world position, and
	# a solver that forgot to_local() would pass this test by accident - which is
	# why the world position is checked as well as the local one.
	var goal := Vector3(1.2, 0.6, 0.7)
	var locals: Array[Vector3] = []
	for offset in [Vector3.ZERO, Vector3(10, 20, 30), Vector3(-5, 0, 100)]:
		var skeleton := _make_skeleton(offset)
		_add_chain(skeleton, TIP, goal)
		skeleton.force_update_all_bone_transforms()
		# The effector is a child of the modifier, which is a child of the
		# skeleton, so its world position moves with the skeleton.
		var modifier := skeleton.get_child(0) as FabrikModifier3D
		modifier.solve_now()
		skeleton.force_update_all_bone_transforms()
		var local := skeleton.get_bone_global_pose(TIP).origin
		locals.append(local)
		var world := skeleton.to_global(local)
		if world.distance_to(offset + goal) > 0.02:
			_fail("at offset " + str(offset) + " the leaf world position is " + str(world)
				+ ", expected " + str(offset + goal))
		skeleton.queue_free()
	for i in range(1, locals.size()):
		if not locals[i].is_equal_approx(locals[0]):
			_fail("skeleton offset changed the solved pose: " + str(locals[0])
				+ " vs " + str(locals[i]))
	if failures == 0:
		print("PASS solved pose is skeleton-local and identical at three skeleton transforms")

func _segment_lengths_survive() -> void:
	var skeleton := _make_skeleton()
	_add_chain(skeleton, TIP, Vector3(1.0, 1.2, 0.0))
	var modifier := skeleton.get_child(0) as FabrikModifier3D
	skeleton.force_update_all_bone_transforms()
	# Adjacent bones, NOT the root-to-tip distance: a chain that bends brings its
	# two ends closer together, and measuring the ends would flag a correct solve
	# as a length change. The first version of this test did exactly that and
	# reported 2.0 -> 1.02 for a solve that was in fact correct.
	var before := []
	for bone in [1, 2]:
		before.append(skeleton.get_bone_global_pose(bone).origin.distance_to(skeleton.get_bone_global_pose(bone + 1).origin))
	modifier.solve_now()
	skeleton.force_update_all_bone_transforms()
	var after := []
	for bone in [1, 2]:
		after.append(skeleton.get_bone_global_pose(bone).origin.distance_to(skeleton.get_bone_global_pose(bone + 1).origin))
	# The chain root is bone 1, which the solve must not move.
	var root_before := Vector3(0, 1, 0)
	var root_now := skeleton.get_bone_global_pose(CHAIN_ROOT).origin
	if not root_now.is_equal_approx(root_before):
		_fail("the chain root moved from " + str(root_before) + " to " + str(root_now))
	var drifted := -1
	for i in 2:
		if absf(before[i] - after[i]) > 0.001:
			drifted = i
	if drifted >= 0:
		_fail("segment " + str(drifted) + " length drifted: " + str(before[drifted]) + " -> " + str(after[drifted]))
	elif modifier.get_last_statuses() != PackedInt32Array([0]):
		_fail("expected every chain to report OK, got " + str(modifier.get_last_statuses()))
	else:
		print("PASS chain root stays put and both segment lengths hold (" + str(after) + ")")
	skeleton.queue_free()

func _transform_modes_differ() -> void:
	# POSITION_ONLY keeps the rotation the solve produced; PRESERVE_ROTATION
	# puts the leaf's pre-solve rotation back; STRAIGHTEN_CHAIN gives the leaf
	# an identity pose rotation; FULL_TRANSFORM takes the leaf's orientation
	# from the effector node. Each is checked against the others, because
	# "they all look plausible" is not a test.
	var results := {}
	for mode in [FabrikEffector.POSITION_ONLY, FabrikEffector.PRESERVE_ROTATION,
			FabrikEffector.STRAIGHTEN_CHAIN, FabrikEffector.FULL_TRANSFORM]:
		var skeleton := _make_skeleton()
		# Tilt the leaf first, so PRESERVE_ROTATION has something to preserve.
		skeleton.set_bone_pose_rotation(TIP, Quaternion(Vector3(0, 0, 1), 0.6))
		var made := _add_chain(skeleton, TIP, Vector3(1.0, 1.4, 0.0))
		var modifier: FabrikModifier3D = made[0]
		var effector: FabrikEffector = made[1]
		effector.transform_mode = mode
		if mode == FabrikEffector.FULL_TRANSFORM:
			# A recognisable orientation, so "took the node's rotation" is
			# checkable and not just "changed somehow".
			effector.basis = Basis(Quaternion(Vector3(1, 0, 0), 0.9))
		skeleton.force_update_all_bone_transforms()
		var rest_basis := skeleton.get_bone_global_pose(TIP).basis.orthonormalized()
		modifier.solve_now()
		skeleton.force_update_all_bone_transforms()
		var leaf_basis := skeleton.get_bone_global_pose(TIP).basis.orthonormalized()
		var leaf_pose_basis := skeleton.get_bone_pose(TIP).basis.orthonormalized()
		# Every mode must still get the leaf to the right PLACE. A mode that
		# fixed the rotation by refusing to move anything would pass the
		# rotation checks and fail this one.
		var place := skeleton.get_bone_global_pose(TIP).origin.distance_to(Vector3(1.0, 1.4, 0.0))
		results[mode] = {
			"place": place,
			"moved": not rest_basis.is_equal_approx(leaf_basis),
			"preserved": rest_basis.is_equal_approx(leaf_basis),
			"leaf_pose_is_identity": leaf_pose_basis.is_equal_approx(Basis()),
			"orientation": leaf_basis.get_rotation_quaternion().angle_to(
				Quaternion(Basis(Quaternion(Vector3(1, 0, 0), 0.9)))),
		}
		skeleton.queue_free()

	for mode in results:
		if float(results[mode]["place"]) > 0.05:
			_fail("transform mode " + str(mode) + " did not solve the position: off by " + str(results[mode]["place"]))

	if results[FabrikEffector.PRESERVE_ROTATION]["preserved"] != true:
		_fail("PRESERVE_ROTATION changed the leaf's rotation")
	if results[FabrikEffector.POSITION_ONLY]["preserved"] == true:
		_fail("POSITION_ONLY did not rotate the leaf; it behaved like PRESERVE_ROTATION")
	if results[FabrikEffector.STRAIGHTEN_CHAIN]["leaf_pose_is_identity"] != true:
		_fail("STRAIGHTEN_CHAIN left a rotation on the leaf pose")
	if float(results[FabrikEffector.FULL_TRANSFORM]["orientation"]) > 0.05:
		_fail("FULL_TRANSFORM did not take the effector's orientation (off by " + str(results[FabrikEffector.FULL_TRANSFORM]["orientation"]) + " rad)")
	if failures == 0:
		print("PASS the four transform modes behave differently as documented")

func _unusable_chains_are_reported() -> void:
	# A chain of one bone has nothing to bend, and a bone with no parent has
	# nowhere to bend from. Both are refused with a reason instead of quietly
	# solving nothing.
	var skeleton := _make_skeleton()
	var made := _add_chain(skeleton, TIP, Vector3(1, 0, 0))
	var modifier: FabrikModifier3D = made[0]
	var effector: FabrikEffector = made[1]
	skeleton.force_update_all_bone_transforms()
	modifier.solve_now()
	if modifier.get_last_error() != "":
		_fail("a healthy chain reported an error: ", modifier.get_last_error())

	effector.chain_length = 1
	skeleton.reset_bone_poses()
	skeleton.force_update_all_bone_transforms()
	modifier.solve_now()
	if modifier.get_last_chain_count() != 0 or modifier.get_last_error() == "":
		_fail("a one-bone chain was not refused with a reason (count " + str(modifier.get_last_chain_count()) + ", error '" + modifier.get_last_error() + "')")

	effector.chain_length = CHAIN_LEN
	effector.bone_name = "root" # a bone with no parent
	skeleton.force_update_all_bone_transforms()
	modifier.solve_now()
	if modifier.get_last_chain_count() != 0 or modifier.get_last_error() == "":
		_fail("a root-bone chain was not refused with a reason")

	effector.bone_name = "no_such_bone"
	skeleton.force_update_all_bone_transforms()
	modifier.solve_now()
	if modifier.get_last_chain_count() != 0 or modifier.get_last_error() == "":
		_fail("an unknown bone name was not refused with a reason")
	if failures == 0:
		print("PASS unusable chains are refused with a reason, not silently dropped")
	skeleton.queue_free()
