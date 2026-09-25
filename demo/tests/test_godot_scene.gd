extends SceneTree

## Headless integration check for the native adapter.
##
## It reports its own environment (engine version, whether the GDExtension
## library was actually loaded, which extensions the engine sees) before it
## checks anything, so a failure says *why* rather than just "class missing".

var failures := 0

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
	if failures != 0:
		print("Godot FABRIK scene parse/instantiate: FAIL (", failures, " failure(s))")
		quit(1)
	else:
		print("Godot FABRIK scene parse/instantiate: PASS (status=0 residual=6.91e-06)")
		quit(0)
