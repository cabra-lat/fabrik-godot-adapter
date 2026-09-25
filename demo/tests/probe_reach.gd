## Diagnose the reach reporting itself.
##
## The motion probe showed the spine reporting UNREACHABLE on 120/120 frames
## while an arm whose target sits 1.4 m from a 0.58 m reach reported OK on
## 0/120. One of those two is wrong, and it is a core question, not a demo
## question. This prints, per chain: the status CODE, the status NAME, the
## root-to-target distance, the chain's reach, and the residual, so the
## disagreement is visible instead of inferred.
extends SceneTree

const FRAMES := 120
const DELTA := 1.0 / 24.0

func _initialize() -> void:
	await process_frame
	var packed: PackedScene = load("res://humanoid_demo.tscn")
	if packed == null:
		printerr("FAIL could not load the humanoid demo")
		quit(1)
		return
	var demo: Node = packed.instantiate()
	root.add_child(demo)
	await process_frame

	# Cross-check the solver's verdict against plain geometry: for an anchored
	# chain the target is unreachable exactly when root->target exceeds the total
	# segment length. Any disagreement is a core bug, so count them.
	var mismatches := 0
	var checked := 0
	# Bone integrity under the demo's REAL motion: the humanoid test drives its
	# own targets, so it would not notice stretching that only this path causes.
	var worst_length_error := 0.0
	var worst_tip_stretch := 0.0
	var counts := {}
	for entry in _chains(demo):
		counts[entry["name"]] = {"unreachable": 0, "geometric": 0, "frames": 0, "maxdist": 0.0, "reach": 0.0}
	for frame in FRAMES:
		demo.call("_process", DELTA)
		for entry in _chains(demo):
			var chain: FabrikChain3D = entry["chain"]
			var joints: PackedVector3Array = chain.joints
			var reach := 0.0
			for length in chain.segment_lengths:
				reach += length
			var distance: float = joints[0].distance_to(chain.target)
			var solver_says := chain.get_last_status_name() == "UNREACHABLE"
			var geometry_says := distance > reach + 0.001
			var row: Dictionary = counts[entry["name"]]
			row["frames"] += 1
			if entry["name"] == "ARM.L" and (frame < 3 or frame == 47 or frame == 48):
				print("    t=", "%.3f" % demo.get("elapsed"),
					" ARM.L target=", chain.target,
					" root=", joints[0],
					" dist=", "%.3f" % distance)
			row["maxdist"] = maxf(row["maxdist"], distance)
			row["reach"] = reach
			if solver_says:
				row["unreachable"] += 1
			if geometry_says:
				row["geometric"] += 1
			# Measured bone length vs declared, and tip distance vs total reach.
			var measured := 0.0
			for seg in chain.segment_lengths.size():
				var actual: float = joints[seg].distance_to(joints[seg + 1])
				measured += actual
				worst_length_error = maxf(worst_length_error, absf(actual - chain.segment_lengths[seg]))
			worst_tip_stretch = maxf(worst_tip_stretch, measured - reach)
			checked += 1
			if solver_says != geometry_says:
				mismatches += 1
				if mismatches <= 5:
					print("MISMATCH ", entry["name"], " frame ", frame,
						" solver=", chain.get_last_status_name(),
						" dist=", "%.4f" % distance, " reach=", "%.4f" % reach,
						" residual=", "%.5f" % chain.get_last_residual())
	for name in counts:
		var row: Dictionary = counts[name]
		print("  ", name, ": solver UNREACHABLE ", row["unreachable"], "/", row["frames"],
			"   geometrically out of reach ", row["geometric"], "/", row["frames"],
			"   max dist ", "%.3f" % row["maxdist"], " reach ", "%.3f" % row["reach"])
	print("demo elapsed after the loop: ", demo.get("elapsed"),
		"  (expected about ", "%.2f" % (FRAMES * DELTA), ")")
	print("demo is_processing: ", demo.is_processing())
	print("checked ", checked, " chain-frames, mismatches: ", mismatches)
	print("worst single-bone length error: ", "%.6f" % worst_length_error, " m")
	print("worst total-length excess over reach: ", "%.6f" % worst_tip_stretch, " m")
	quit(1 if mismatches > 0 else 0)

func _unused(demo: Node) -> void:
	for frame in FRAMES:
		demo.call("_process", DELTA)
		print("--- frame ", frame)
		for entry in _chains(demo):
			var chain: FabrikChain3D = entry["chain"]
			var joints: PackedVector3Array = chain.joints
			var reach := 0.0
			for length in chain.segment_lengths:
				reach += length
			var distance: float = joints[0].distance_to(chain.target)
			print("  ", entry["name"],
				"  code=", chain.get_last_status(),
				"  name=", chain.get_last_status_name(),
				"  root->target=", "%.3f" % distance,
				"  reach=", "%.3f" % reach,
				"  residual=", "%.5f" % chain.get_last_residual(),
				"  anchored=", chain.root_anchored,
				"  tip->target=", "%.3f" % joints[joints.size() - 1].distance_to(chain.target))
	quit(0)

func _chains(demo: Node) -> Array:
	var out: Array = []
	out.append({"name": "SPINE", "chain": demo.get("_spine").chain})
	out.append({"name": "HEAD ", "chain": demo.get("_head").chain})
	for limb in demo.get("_limbs"):
		out.append({"name": limb.label, "chain": limb.chain})
	return out
