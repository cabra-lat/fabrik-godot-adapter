--!strict
## Frame-grabbing runner for the visual demo.
##
## Renders the demo scene deterministically and saves PNG frames, so a GIF or a
## video can be produced from them afterwards. Rendering runs on a fixed frame
## step with a fixed number of frames, so the output is reproducible.
##
## Usage (from the repository root):
##   godot --path demo --script res://tests/render_frames.gd -- <out_dir> [frames] [fps]
extends SceneTree

const DEFAULT_OUT := "user://frames"
const DEFAULT_FRAMES := 96
const FPS := 24

var _out_dir := DEFAULT_OUT
var _frames := DEFAULT_FRAMES
var _index := 0

func _initialize() -> void:
	var args := OS.get_cmdline_user_args()
	if args.size() >= 1:
		_out_dir = args[0]
	if args.size() >= 2:
		_frames = int(args[1])
	DirAccess.make_dir_recursive_absolute(_out_dir)
	print("render: ", _frames, " frames -> ", ProjectSettings.globalize_path(_out_dir))
	var packed := load("res://visual_demo.tscn")
	if packed == null:
		printerr("render: could not load the visual demo scene")
		quit(1)
		return
	var scene := packed.instantiate()
	root.add_child(scene)
	var camera := _find_camera(scene)
	if camera == null:
		printerr("render: the demo scene has no camera")
		quit(1)
		return
	camera.make_current()
	# Fixed step: the demo animates from its own clock, so pin it explicitly.
	Engine.max_fps = 0
	await _render_sequence(scene)

func _find_camera(node: Node) -> Camera3D:
	if node is Camera3D:
		return node as Camera3D
	for child in node.get_children():
		var found := _find_camera(child)
		if found != null:
			return found
	return null

func _render_sequence(scene: Node) -> void:
	var image := Image.create_empty(960, 540, false, Image.FORMAT_RGB8)
	for frame in _frames:
		# Drive the demo by hand so the frame count is exact.
		scene._process(1.0 / float(FPS))
		await process_frame
		await RenderingServer.frame_post_draw
		var viewport := get_root().get_texture().get_image()
		if viewport == null:
			printerr("render: no viewport image at frame ", frame)
			quit(1)
			return
		image = viewport.duplicate() as Image
		image.convert(Image.FORMAT_RGB8)
		var path := _out_dir.path_join("frm_%04d.png" % frame)
		var error := image.save_png(path)
		if error != OK:
			printerr("render: could not save ", path, " error ", error)
			quit(1)
			return
		_index += 1
		if frame % 12 == 0:
			print("  frame ", frame, "/", _frames)
	print("render: wrote ", _index, " frames")
	quit(0)
