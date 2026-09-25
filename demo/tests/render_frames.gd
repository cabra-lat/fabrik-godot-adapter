## Frame-grabbing runner for the visual demo.
##
## Renders the demo scene deterministically and saves PNG frames, so a GIF or a
## video can be produced from them afterwards. The frame count is exact and the
## demo is stepped by a fixed delta, so the output is reproducible.
##
## Usage (from the repository root):
##   godot --path demo --script res://tests/render_frames.gd -- <out_dir> [frames]
##
## Needs a real GL context: a headless run has no viewport image to grab.
extends SceneTree

const SCENE := "res://visual_demo.tscn"
const DEFAULT_OUT := "user://frames"
const DEFAULT_FRAMES := 96
const FPS := 24

var _out_dir := DEFAULT_OUT
var _frames := DEFAULT_FRAMES
var _written := 0
var _scene: Node = null
var _target := Vector3.ZERO
var _residual := 0.0
var _status := 0

func _initialize() -> void:
	var args := OS.get_cmdline_user_args()
	if args.size() >= 1:
		_out_dir = args[0]
	if args.size() >= 2:
		_frames = maxi(1, int(args[1]))
	DirAccess.make_dir_recursive_absolute(_out_dir)
	print("render: ", _frames, " frames -> ", ProjectSettings.globalize_path(_out_dir))
	var packed: PackedScene = load(SCENE)
	if packed == null:
		printerr("render: could not load ", SCENE)
		quit(1)
		return
	_scene = packed.instantiate()
	root.add_child(_scene)
	# _ready() only runs once the tree starts processing, which has not happened
	# yet during _initialize(). Wait for a frame or the scene is still empty.
	await process_frame
	var camera := _find_camera(_scene)
	if camera == null:
		printerr("render: no camera in the scene; children: ", _describe(_scene, 0))
		quit(1)
		return
	camera.make_current()
	await _render_sequence()

func _find_camera(node: Node) -> Camera3D:
	if node is Camera3D:
		return node as Camera3D
	for child in node.get_children():
		var found := _find_camera(child)
		if found != null:
			return found
	return null

func _describe(node: Node, depth: int) -> String:
	var out := "\n"
	for child in node.get_children():
		out += "  ".repeat(depth + 1) + child.get_class() + " (" + str(child.name) + ")\n"
		if depth < 3:
			out += _describe(child, depth + 1)
	return out

func _render_sequence() -> void:
	for frame in _frames:
		# Drive the demo by hand so the frame count is exact and reproducible
		# instead of depending on wall-clock time.
		if _scene.has_method("_process"):
			_scene.call("_process", 1.0 / float(FPS))
		await process_frame
		await RenderingServer.frame_post_draw
		var image := get_root().get_texture().get_image()
		if image == null:
			printerr("render: no viewport image at frame ", frame)
			quit(1)
			return
		if image.get_width() < 2:
			printerr("render: viewport image is empty (no GL context?)")
			quit(1)
			return
		image.convert(Image.FORMAT_RGB8)
		var path := _out_dir.path_join("frm_%04d.png" % frame)
		if image.save_png(path) != OK:
			printerr("render: could not save ", path)
			quit(1)
			return
		_written += 1
		if frame % 12 == 0:
			print("  frame ", frame, "/", _frames)
	print("render: wrote ", _written, " frames to ", ProjectSettings.globalize_path(_out_dir))
	quit(0)
