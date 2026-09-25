extends SceneTree

var failures := 0

func _initialize() -> void:
	var packed := load("res://demo/fabrik_demo.tscn")
	if packed == null:
		failures += 1
		print("FAIL scene parse")
		quit(1)
		return
	var instance = packed.instantiate()
	if instance == null:
		failures += 1
		print("FAIL scene instantiate")
		quit(1)
		return
	root.add_child(instance)
	await process_frame
	if instance.solve_status != 0:
		failures += 1
		print("FAIL adapter solve status=", instance.solve_status)
	if instance.solve_residual > 0.00001:
		failures += 1
		print("FAIL adapter residual=", instance.solve_residual)
	if failures != 0:
		quit(1)
	else:
		print("Godot FABRIK scene parse/instantiate: PASS (status=0 residual=", instance.solve_residual, ")")
		quit(0)
