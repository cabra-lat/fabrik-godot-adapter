extends SceneTree

## Print what the engine reports about itself, and which GDExtension descriptor
## tags it accepts.
##
## A GDExtension that fails to load produces no useful error: the class is never
## registered and the suites die at GDScript parse time with "Identifier
## FabrikChain3D not declared", which points at the test script rather than at
## the descriptor. Godot selects the library by splitting each [libraries] key
## on "." and requiring every part to be a satisfied feature tag
## (GDExtensionLibraryLoader::find_extension_library), with the most-parts match
## winning, so the fix is always in this list.
##
## Run before the suites. It exits non-zero only when no [libraries] key is
## usable for this runner, which is the one condition that makes every later
## step fail for an unrelated-looking reason. A green run still leaves the full
## environment on the record.

func _initialize() -> void:
	print("--- engine environment")
	print("engine version:   ", Engine.get_version_info()["string"])
	print("OS.get_name():    ", OS.get_name())
	print("architecture:     ", Engine.get_architecture_name())
	print("processor:        ", OS.get_processor_name())
	print("editor hint:      ", Engine.is_editor_hint())
	print("--- descriptor tags this engine accepts")
	for tag: String in [
		"linux", "windows", "macos",
		"debug", "release", "template", "template_release", "template_debug",
		"editor", "editor_hint",
		"x86_64", "x86_32", "x86", "arm64", "arm32", "arm", "universal", "64",
	]:
		print("  ", tag, " = ", OS.has_feature(tag))
	print("--- extension status")
	print("descriptor present: ", FileAccess.file_exists("res://bin/fabrik_adapter.gdextension"))
	var config := ConfigFile.new()
	var err := config.load("res://bin/fabrik_adapter.gdextension")
	if err != OK:
		print("FAIL: descriptor did not parse, error ", err)
		quit(1)
		return
	print("FabrikChain3D registered: ", ClassDB.class_exists("FabrikChain3D"))
	print("loaded extensions: ", GDExtensionManager.get_loaded_extensions())
	print("descriptor [libraries] keys:")
	var usable: Array[String] = []
	for key: String in config.get_section_keys("libraries"):
		var all_met := true
		for part: String in key.split("."):
			if not OS.has_feature(part):
				all_met = false
		if all_met:
			usable.append(key)
		print("  ", key, "  ->  ", ("USABLE" if all_met else "unusable"),
			"  ", config.get_value("libraries", key))
	if usable.is_empty():
		print("FAIL: no [libraries] key matches this runner.")
		print("      A descriptor key is a list of feature tags joined by '.', and every")
		print("      part must be a feature this engine reports. A key can be present in")
		print("      the file and still be unusable, so grepping the text proves nothing;")
		print("      the extension must also actually be registered below.")
		quit(1)
		return
	print("usable on this runner: ", ", ".join(usable))
	if not ClassDB.class_exists("FabrikChain3D"):
		print("FAIL: a usable key exists but FabrikChain3D is not registered, so the")
		print("      library was selected and then failed to load.")
		quit(1)
		return
	quit(0)
