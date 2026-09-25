## Does root anchoring actually hold?
##
## The reach probe showed an anchored arm chain whose ROOT translated along with
## its target, keeping root->target at a constant 0.300 m. If the root is not
## pinned, "anchored" is a lie and every reach computation built on it is wrong.
## This is the smallest case that can settle it: one chain, one solve, a target
## moved far away, and the root printed before and after.
extends SceneTree

func _initialize() -> void:
	var chain := FabrikChain3D.new()
	chain.joints = PackedVector3Array([Vector3(0, 1, 0), Vector3(0, 0.7, 0), Vector3(0, 0.4, 0)])
	chain.segment_lengths = PackedFloat32Array([0.30, 0.30])
	chain.root_anchored = true
	chain.tolerance = 0.00001
	chain.max_iterations = 64

	print("anchored = ", chain.root_anchored)
	var before: PackedVector3Array = chain.joints
	print("root before solve: ", before[0])

	# Target well beyond the 0.60 m reach, off to the side.
	chain.target = Vector3(2.0, 1.0, 0.0)
	var status := chain.solve()
	var after: PackedVector3Array = chain.joints
	print("status: ", status, " ", chain.get_last_status_name(), " residual ", chain.get_last_residual())
	print("root after  solve: ", after[0])
	print("root moved by: ", after[0].distance_to(before[0]))

	# Repeat with anchoring off: the root is then EXPECTED to travel.
	var free_chain := FabrikChain3D.new()
	free_chain.joints = PackedVector3Array([Vector3(0, 1, 0), Vector3(0, 0.7, 0), Vector3(0, 0.4, 0)])
	free_chain.segment_lengths = PackedFloat32Array([0.30, 0.30])
	free_chain.root_anchored = false
	free_chain.target = Vector3(2.0, 1.0, 0.0)
	free_chain.solve()
	var free_after: PackedVector3Array = free_chain.joints
	print("unanchored root moved by: ", free_after[0].distance_to(Vector3(0, 1, 0)))

	# And once more anchored, with smoothing on, since the rebuild path could
	# move the root even when the core does not.
	var eased := FabrikChain3D.new()
	eased.joints = PackedVector3Array([Vector3(0, 1, 0), Vector3(0, 0.7, 0), Vector3(0, 0.4, 0)])
	eased.segment_lengths = PackedFloat32Array([0.30, 0.30])
	eased.root_anchored = true
	eased.smoothing = 0.3
	var drift := 0.0
	for i in 20:
		eased.target = Vector3(1.0 + 0.1 * float(i), 1.0, 0.3 * float(i))
		eased.solve()
		drift = maxf(drift, eased.joints[0].distance_to(Vector3(0, 1, 0)))
	print("anchored+smoothing: worst root drift over 20 solves: ", drift)

	# Now the same question about the DEMO's own arm chain, which is the one that
	# appeared to drift. Print what it actually declares.
	var packed: PackedScene = load("res://humanoid_demo.tscn")
	var demo: Node = packed.instantiate()
	root.add_child(demo)
	await process_frame
	for limb in demo.get("_limbs"):
		var demo_chain: FabrikChain3D = limb.chain
		print(limb.label, ": joints=", demo_chain.joints.size(),
			" lengths=", demo_chain.segment_lengths,
			" anchored=", demo_chain.root_anchored,
			" smoothing=", demo_chain.smoothing,
			" limb.root=", limb.root,
			" chain.joints[0]=", demo_chain.joints[0])
		# Frame-by-frame root trace for the demo's left arm, against the shoulder.
	var arm = demo.get("_limbs")[0]
	var shoulder: Vector3 = arm.root
	print("--- root trace, ARM.L (shoulder ", shoulder, ")")
	for i in 8:
		demo.call("_process", 1.0 / 24.0)
		var c: FabrikChain3D = arm.chain
		print("  frame ", i, " t=", "%.3f" % demo.get("elapsed"),
			" root=", c.joints[0],
			" drift=", "%.4f" % c.joints[0].distance_to(shoulder),
			" anchored=", c.root_anchored,
			" target=", c.target)
		# Reproduce the demo's exact sequence: a first solve with the DEFAULT target
	# (the demo calls _solve_once() in _ready before any target is set), then
	# real targets, with smoothing on. The isolated chains above never took this
	# path, and they never drifted.
	print("--- demo sequence reproduced in isolation")
	var seq := FabrikChain3D.new()
	seq.joints = PackedVector3Array([Vector3(-0.2, 1.35, 0), Vector3(-0.2, 1.05, 0), Vector3(-0.2, 0.77, 0)])
	seq.segment_lengths = PackedFloat32Array([0.30, 0.28])
	seq.root_anchored = true
	seq.smoothing = 0.25
	seq.tolerance = 0.00001
	seq.max_iterations = 64
	# _ready(): solve once with the target still at its default.
	seq.solve()
	print("  after default-target solve: root=", seq.joints[0], " status=", seq.get_last_status_name())
	for i in 6:
		seq.target = Vector3(-0.215 + 0.006 * float(i), 1.30 + 0.005 * float(i), -0.466 - 0.009 * float(i))
		seq.solve()
		print("  frame ", i, " root=", seq.joints[0],
			" drift=", "%.4f" % seq.joints[0].distance_to(Vector3(-0.2, 1.35, 0)),
			" status=", seq.get_last_status_name())
	# Same again with smoothing disabled, to separate the two paths.
	print("--- same sequence, smoothing = 0")
	var raw := FabrikChain3D.new()
	raw.joints = PackedVector3Array([Vector3(-0.2, 1.35, 0), Vector3(-0.2, 1.05, 0), Vector3(-0.2, 0.77, 0)])
	raw.segment_lengths = PackedFloat32Array([0.30, 0.28])
	raw.root_anchored = true
	raw.tolerance = 0.00001
	raw.max_iterations = 64
	raw.solve()
	for i in 6:
		raw.target = Vector3(-0.215 + 0.006 * float(i), 1.30 + 0.005 * float(i), -0.466 - 0.009 * float(i))
		raw.solve()
		print("  frame ", i, " root=", raw.joints[0],
			" drift=", "%.4f" % raw.joints[0].distance_to(Vector3(-0.2, 1.35, 0)))
	quit(0)
