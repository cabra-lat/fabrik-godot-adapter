extends Node3D

## Visual demo: one FABRIK chain chasing a moving target.
##
## Deliberately small. Geometry is built in code, so there is nothing to import
## and the whole thing is one script plus one scene:
##   godot --path demo res://visual_demo.tscn
##
## Bones and joints are MultiMeshes: one draw call each, and per-instance
## transforms let a 7-bone chain be posed from the adapter's rotations without
## one Node per bone.

const JOINT_COUNT := 7
const SEGMENT := 0.55

var chain: FabrikChain3D
var label: Label
var elapsed := 0.0
var _bones: MultiMeshInstance3D
var _joints: MultiMeshInstance3D
var _target: MeshInstance3D
var _last_status := 0

func _ready() -> void:
	_build_world()
	_build_chain()
	_build_overlay()
	chain.solve()
	set_process(true)

func _build_world() -> void:
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-50.0, -35.0, 0.0)
	light.light_energy = 1.2
	add_child(light)

	var camera := Camera3D.new()
	camera.position = Vector3(0.0, 2.4, 5.4)
	camera.look_at_from_position(camera.position, Vector3(0.0, 1.4, 0.0), Vector3.UP)
	add_child(camera)

	var floor_instance := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(12.0, 12.0)
	var floor_material := StandardMaterial3D.new()
	floor_material.albedo_color = Color(0.16, 0.17, 0.20)
	floor_instance.mesh = plane
	floor_instance.mesh.surface_set_material(0, floor_material)
	add_child(floor_instance)

func _build_chain() -> void:
	chain = FabrikChain3D.new()
	chain.tolerance = 0.0005
	chain.max_iterations = 96
	# A little smoothing so the demo reads as motion, not teleporting.
	chain.smoothing = 0.35

	var points := PackedVector3Array()
	var lengths := PackedFloat32Array()
	for i in JOINT_COUNT:
		points.append(Vector3(0.0, 0.1 + SEGMENT * i, 0.0))
		if i > 0:
			lengths.append(SEGMENT)
	chain.joints = points
	chain.segment_lengths = lengths
	chain.root_anchored = true
	chain.target = Vector3(1.2, 2.0, 0.0)
	chain.solve_finished.connect(_on_solve_finished)

	# One unit cylinder per segment, along +Y, scaled per instance.
	_bones = MultiMeshInstance3D.new()
	var bone_multimesh := MultiMesh.new()
	bone_multimesh.transform_format = MultiMesh.TRANSFORM_3D
	bone_multimesh.use_colors = true
	var cylinder := CylinderMesh.new()
	cylinder.top_radius = 0.055
	cylinder.bottom_radius = 0.055
	cylinder.height = 1.0
	cylinder.radial_segments = 12
	var bone_material := StandardMaterial3D.new()
	bone_material.albedo_color = Color(0.42, 0.72, 0.95)
	bone_material.roughness = 0.35
	cylinder.material = bone_material
	bone_multimesh.mesh = cylinder
	bone_multimesh.instance_count = JOINT_COUNT - 1
	_bones.multimesh = bone_multimesh
	add_child(_bones)

	_joints = MultiMeshInstance3D.new()
	var joint_multimesh := MultiMesh.new()
	joint_multimesh.transform_format = MultiMesh.TRANSFORM_3D
	joint_multimesh.use_colors = true
	var sphere := SphereMesh.new()
	sphere.radius = 0.075
	sphere.height = 0.15
	sphere.radial_segments = 12
	sphere.rings = 6
	joint_multimesh.mesh = sphere
	joint_multimesh.instance_count = JOINT_COUNT
	_joints.multimesh = joint_multimesh
	add_child(_joints)

	_target = MeshInstance3D.new()
	var target_mesh := SphereMesh.new()
	target_mesh.radius = 0.12
	target_mesh.height = 0.24
	target_mesh.radial_segments = 16
	target_mesh.rings = 8
	var target_material := StandardMaterial3D.new()
	target_material.albedo_color = Color(0.95, 0.42, 0.35)
	target_material.emission_enabled = true
	target_material.emission = Color(0.45, 0.12, 0.06)
	_target.mesh = target_mesh
	add_child(_target)

func _build_overlay() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	label = Label.new()
	label.position = Vector2(16.0, 12.0)
	label.add_theme_font_size_override("font_size", 16)
	label.add_theme_color_override("font_color", Color(0.92, 0.94, 0.98))
	layer.add_child(label)

func _on_solve_finished(status: int, residual: float) -> void:
	_last_status = status
	if status != 0:
		label.text = "FABRIK  status %d  residual %.6f" % [status, residual]
	else:
		label.text = "FABRIK  joints %d  residual %.6f  smoothing %.2f" % [
			JOINT_COUNT, residual, chain.smoothing]

func _process(delta: float) -> void:
	elapsed += delta
	# Lissajous path so the chain has to bend in two axes at once.
	var t := elapsed
	chain.target = Vector3(
		sin(t * 0.9) * 1.5,
		1.7 + sin(t * 0.6) * 1.1,
		cos(t * 1.3) * 1.2)
	chain.solve()
	_target.position = chain.target
	_draw_chain()

func _draw_chain() -> void:
	var points := chain.joints
	var rotations := chain.get_last_joint_rotations()
	var segments := mini(points.size() - 1, rotations.size())
	for i in segments:
		var a := points[i]
		var b := points[i + 1]
		var length := a.distance_to(b)
		if length <= 0.0001:
			continue
		# The adapter reports bones pointing along local +Y and the cylinder is
		# one unit tall along +Y, so the rotation maps straight onto the segment.
		var basis := Basis(rotations[i]).scaled(Vector3(1.0, length, 1.0))
		_bones.multimesh.set_instance_transform(i, Transform3D(basis, (a + b) * 0.5))
		_bones.multimesh.set_instance_color(i, Color(0.42, 0.72, 0.95))
	for i in points.size():
		_joints.multimesh.set_instance_transform(i, Transform3D(Basis(), points[i]))
		var tint := Color(0.95, 0.78, 0.35) if i == points.size() - 1 else Color(0.78, 0.84, 0.92)
		_joints.multimesh.set_instance_color(i, tint)
