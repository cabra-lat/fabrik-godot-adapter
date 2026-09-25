## Ground-truth motion probe: no images involved.
##
## Image review is not available in every environment, and a pixel-diff proxy
## measures the torso silhouette rather than the solve. This drives the humanoid
## demo exactly as the renderer does and reports how far each chain's end
## effector actually travels, plus the reach statistics, straight from the
## solver output.
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

	var names := PackedStringArray()
	var min_travel := {}
	var max_travel := {}
	var unreachable := {}
	var converged := {}
	for chain: FabrikChain3D in _chains(demo):
		names.append("")
		min_travel[chain] = Vector3(INF, INF, INF)
		max_travel[chain] = Vector3(-INF, -INF, -INF)
		unreachable[chain] = 0
		converged[chain] = 0

	for frame in FRAMES:
		demo.call("_process", DELTA)
		for chain: FabrikChain3D in _chains(demo):
			var tip: Vector3 = chain.joints[chain.joints.size() - 1]
			var lo: Vector3 = min_travel[chain]
			var hi: Vector3 = max_travel[chain]
			lo.x = minf(lo.x, tip.x); lo.y = minf(lo.y, tip.y); lo.z = minf(lo.z, tip.z)
			hi.x = maxf(hi.x, tip.x); hi.y = maxf(hi.y, tip.y); hi.z = maxf(hi.z, tip.z)
			min_travel[chain] = lo
			max_travel[chain] = hi
			if chain.get_last_status_name() == "UNREACHABLE":
				unreachable[chain] += 1
			else:
				converged[chain] += 1

	print("end-effector travel over ", FRAMES, " frames (", "%.1f" % (FRAMES * DELTA), " s):")
	var index := 0
	for chain: FabrikChain3D in _chains(demo):
		var lo: Vector3 = min_travel[chain]
		var hi: Vector3 = max_travel[chain]
		var travel := (hi - lo).length()
		print("  chain ", index, ": travel ", "%.3f" % travel,
			" m   x[", "%.2f" % lo.x, ",", "%.2f" % hi.x, "]",
			" y[", "%.2f" % lo.y, ",", "%.2f" % hi.y, "]",
			" z[", "%.2f" % lo.z, ",", "%.2f" % hi.z, "]",
			"   UNREACHABLE ", unreachable[chain], "/", FRAMES)
		index += 1
	quit(0)

func _chains(demo: Node) -> Array:
	var out: Array = []
	out.append(demo.get("_spine").chain)
	out.append(demo.get("_head").chain)
	for limb in demo.get("_limbs"):
		out.append(limb.chain)
	return out
