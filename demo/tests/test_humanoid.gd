## Headless assertions for the humanoid demo.
##
## The demo is only worth rendering if it proves the solver does the right thing
## under the conditions a rig actually creates. These are those conditions:
## many chains at once, targets that leave the reachable sphere, and bone
## lengths that must not drift while it happens.
extends SceneTree

const PELVIS := Vector3(0.0, 0.95, 0.0)

var failures := 0

func _initialize() -> void:
	await process_frame
	var packed: PackedScene = load("res://humanoid_demo.tscn")
	if packed == null:
		_fail("could not load the humanoid demo")
		_finish()
		return
	var demo: Node = packed.instantiate()
	root.add_child(demo)
	await process_frame

	# The demo must actually run its chains, not just build meshes.
	_multi_chain(demo)
	_reach_transition(demo)
	_smoothing_stretches_nothing(demo)
	_segments_never_stretch(demo)
	_anchored_roots_never_drift(demo)
	_pose_skeleton_matches(demo)
	_finish()

func _fail(message: String, detail: String = "") -> void:
	failures += 1
	if detail.is_empty():
		printerr("FAIL ", message)
	else:
		printerr("FAIL ", message, " ", detail)

func _limbs(demo: Node) -> Array:
	var limbs: Array = demo.get("_limbs")
	return limbs

## Six chains must be live at once: spine, head, two arms, two legs.
func _multi_chain(demo: Node) -> void:
	var limbs := _limbs(demo)
	if limbs.size() != 4:
		_fail("expected 4 limb chains, got " + str(limbs.size()))
		return
	var spine = demo.get("_spine")
	var head = demo.get("_head")
	if spine == null or head == null:
		_fail("spine/head chains are missing")
		return
	var live := 0
	var chains: Array[FabrikChain3D] = []
	chains.append(spine.chain)
	chains.append(head.chain)
	for limb in limbs:
		chains.append(limb.chain)
	for chain in chains:
		if chain.joints.size() >= 3:
			live += 1
	if live != 6:
		_fail("expected 6 live chains, got " + str(live))
		return
	print("PASS six chains are live at once (spine, head, 2 arms, 2 legs)")

## The point of the demo: a target that starts reachable and ends out of reach
## must flip the status and straighten the chain toward it. A solver that only
## ever sees reachable targets is untested.
func _reach_transition(demo: Node) -> void:
	var limb = _limbs(demo)[0]
	var chain: FabrikChain3D = limb.chain
	var root: Vector3 = chain.joints[0]
	var reach := 0.0
	for length in chain.segment_lengths:
		reach += length

	# Reachable: solve and expect OK with a tiny residual.
	chain.target = root + Vector3(0.0, -reach * 0.8, 0.0)
	chain.solve()
	if chain.get_last_status_name() != "OK":
		_fail("reachable target did not report OK: " + chain.get_last_status_name())
		return
	if chain.get_last_residual() > 0.01:
		_fail("reachable target left a large residual: " + str(chain.get_last_residual()))
		return

	# Out of reach: further than the chain can span. Smoothing is turned off
	# here on purpose - the point of this assertion is the SOLVER's out-of-reach
	# behaviour, and an eased chain is supposed to lag it by design.
	chain.smoothing = 0.0
	chain.target = root + Vector3(0.0, -(reach + 1.5), 0.0)
	chain.solve()
	if chain.get_last_status_name() == "OK":
		_fail("out-of-reach target still reported OK")
		return
	# FABRIK's signature failure mode: the chain straightens and points at the
	# target instead of drifting somewhere arbitrary.
	var joints: PackedVector3Array = chain.joints
	var first := (joints[1] - joints[0]).normalized()
	var last := (joints[joints.size() - 1] - joints[joints.size() - 2]).normalized()
	if first.dot(last) < 0.999:
		_fail("out-of-reach chain did not straighten: segment dot = " + str(first.dot(last)))
		return
	print("PASS reach transition: OK when reachable, straightens and reports " + chain.get_last_status_name() + " when not")

## With smoothing ON, the chain must still converge to the straightened
## out-of-reach pose over time, and must not stretch while it gets there. This is
## the case that caught position-space smoothing shortening a 0.30 m bone.
func _smoothing_stretches_nothing(demo: Node) -> void:
	var limb = _limbs(demo)[0]
	var chain: FabrikChain3D = limb.chain
	chain.smoothing = 0.45
	chain.target = limb.root + Vector3(0.0, -3.0, 0.0)  # far out of reach
	var first := 0.0
	var last := 0.0
	for i in 240:
		chain.solve()
		var joints: PackedVector3Array = chain.joints
		for s in chain.segment_lengths.size():
			var actual := joints[s].distance_to(joints[s + 1])
			var expected: float = chain.segment_lengths[s]
			if absf(actual - expected) > 0.002:
				_fail("smoothing stretched " + limb.label + " segment " + str(s) + ": " + str(actual) + " vs " + str(expected))
				return
		first = joints[0].distance_to(chain.target)
		last = joints[joints.size() - 1].distance_to(chain.target)
	if first <= 0.0 or last <= 0.0:
		_fail("smoothing run produced a degenerate pose")
		return
	if last >= first:
		_fail("smoothing did not converge towards the out-of-reach target: " + str(first) + " -> " + str(last))
		return
	# Converged: the tip is now at the chain's reach, pointing at the target.
	var reach := 0.0
	for length in chain.segment_lengths:
		reach += length
	if absf(last - (chain.joints[0].distance_to(chain.target) - reach)) > 0.01:
		_fail("smoothed chain did not settle fully extended: tip gap " + str(last))
		return
	print("PASS smoothing eases a chain without stretching any bone, and settles fully extended")

## An anchored chain must keep its root exactly where it was given.
##
## This is the assertion that would have caught the core bug: the solver measured
## each backward-pass joint against the target instead of the joint behind it and
## never re-pinned the root, so an anchored chain crept towards its target - the
## demo's arms visibly detached from the shoulders while `root_anchored` still
## reported true. Driving the demo's REAL motion, not synthetic targets, because
## the synthetic case did not reproduce it.
func _anchored_roots_never_drift(demo: Node) -> void:
	var worst := 0.0
	var worst_label := ""
	var chains: Array = [demo.get("_spine"), demo.get("_head")]
	for limb in demo.get("_limbs"):
		chains.append(limb)
	for entry in chains:
		var chain: FabrikChain3D = entry.chain
		var declared_root: Vector3 = entry.root
		var reported := false
		for i in 90:
			demo.call("_process", 1.0 / 24.0)
			var drift: float = chain.joints[0].distance_to(declared_root)
			if drift > worst:
				worst = drift
				worst_label = entry.label
			# Trace the first frame that breaks the invariant, so a failure says
			# when and why rather than only how far.
			if drift > 1.0e-4 and not reported:
				reported = true
				print("    first drift on ", entry.label, " at frame ", i,
					": drift=", drift,
					" anchored=", chain.root_anchored,
					" status=", chain.get_last_status_name(),
					" root=", chain.joints[0],
					" declared=", declared_root,
					" target=", chain.target,
					" smoothing=", chain.smoothing)
	if worst > 1.0e-4:
		_fail("anchored root drifted on " + worst_label + ": " + str(worst) + " m")
		return
	print("PASS anchored roots never drift over 90 frames of real motion")

## Bones that stretch are a rig lying about its own skeleton.
func _segments_never_stretch(demo: Node) -> void:
	for limb in _limbs(demo):
		var chain: FabrikChain3D = limb.chain
		var before := PackedFloat32Array(chain.segment_lengths)
		# Drive several frames across the reach boundary.
		for i in 12:
			limb.chain.target = limb.root + Vector3(
				sin(float(i) * 0.8) * 0.75, -0.6, cos(float(i) * 0.6) * 0.5)
			chain.solve()
		var after: PackedVector3Array = chain.joints
		for i in chain.segment_lengths.size():
			var actual := after[i].distance_to(after[i + 1])
			var expected: float = chain.segment_lengths[i]
			if absf(actual - expected) > 0.002:
				_fail(limb.label + " segment " + str(i) + " length drifted: " + str(actual) + " vs " + str(expected))
				return
	print("PASS segment lengths preserved across reach transitions on all limbs")

## The production-facing path: the same rotations must pose a real Skeleton3D.
func _pose_skeleton_matches(demo: Node) -> void:
	var limb = _limbs(demo)[0]
	limb.chain.target = limb.root + Vector3(0.3, -0.5, 0.2)
	limb.chain.solve()
	var skeleton: Skeleton3D = demo.get("skeleton")
	if skeleton == null:
		_fail("the demo has no Skeleton3D to pose")
		return
	if skeleton.get_bone_count() < 3:
		_fail("the demo skeleton has no bones to pose, so the test proves nothing")
		return
	# Real bone names from the demo skeleton, one per joint of the chain.
	var names := PackedStringArray()
	for i in 3:
		names.append(skeleton.get_bone_name(i))
	var posed: int = limb.chain.pose_skeleton(skeleton, names)
	if posed < 0:
		_fail("pose_skeleton rejected valid bone names: " + str(posed))
		return
	# The poses must actually have been written.
	var changed := false
	for i in 3:
		if skeleton.get_bone_pose_rotation(i) != Quaternion.IDENTITY:
			changed = true
	if not changed:
		_fail("pose_skeleton reported success but wrote no bone rotations")
		return
	# Bones that do not exist must be reported, not silently accepted.
	var bogus := PackedStringArray(["nope", "alsonope", "stillnope"])
	if limb.chain.pose_skeleton(skeleton, bogus) >= 0:
		_fail("pose_skeleton accepted bone names that do not exist")
		return
	# A chain/skeleton size mismatch must be reported too.
	if limb.chain.pose_skeleton(skeleton, PackedStringArray(["only_one"])) >= 0:
		_fail("pose_skeleton accepted a bone list that does not match the chain")
		return
	print("PASS pose_skeleton writes bones and rejects mismatched bone names")

func _finish() -> void:
	if failures == 0:
		print("Godot FABRIK humanoid: PASS (0 failure(s))")
	else:
		printerr("Godot FABRIK humanoid: FAIL (" + str(failures) + " failure(s))")
	quit(1 if failures > 0 else 0)
