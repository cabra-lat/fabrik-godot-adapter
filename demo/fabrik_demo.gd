extends Node3D

var chain
var solve_status: int = -1
var solve_residual: float = 0.0

func _ready() -> void:
	chain = FabrikChain3D.new()
	chain.joints = PackedVector3Array([
		Vector3(0.0, 0.0, 0.0),
		Vector3(1.0, 0.0, 0.0),
		Vector3(2.0, 0.0, 0.0),
	])
	chain.target = Vector3(1.0, 1.0, 0.0)
	chain.segment_lengths = PackedFloat32Array([1.0, 1.0])
	solve_status = chain.solve()
	solve_residual = chain.get_last_residual()
	print("Godot FABRIK adapter scene: status=", solve_status,
		" name=", chain.get_last_status_name(), " residual=", solve_residual)
