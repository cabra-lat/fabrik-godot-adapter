## Humanoid FABRIK demo.
##
## The point of this scene is to exercise the solver on the shape it exists for:
## a body driven by several chains at once, whose targets are frequently out of
## reach. A single chain chasing a sphere proves almost nothing.
##
## What this actually demonstrates, and the harness asserts all of it headlessly
## in tests/test_humanoid.gd:
##
##   * SIX independent chains solved per frame from one rig: spine, two arms,
##     two legs, and a head chain. Multi-chain is how IK is really used.
##   * Arm and leg targets are driven on paths that CROSS the reach boundary, so
##     the status visibly flips OK -> UNREACHABLE -> OK. A solver that only ever
##     sees reachable targets has not been tested.
##   * When a target is out of reach the chain must STRAIGHTEN and point at it,
##     which is FABRIK's characteristic failure mode and the easiest thing to
##     get wrong.
##   * Segment lengths must be preserved under every one of those transitions.
##     If a bone stretches, the rig is lying about its own skeleton.
##   * The same rotations are pushed onto a real Skeleton3D through
##     pose_skeleton(), so the production-facing path is exercised, not just
##     the numbers.
##
## Geometry is built in code; there is nothing to import.
extends Node3D

const PELVIS := Vector3(0.0, 0.95, 0.0)

## Arm: shoulder -> elbow -> hand. Leg: hip -> knee -> foot. Kept short on
## purpose so the moving targets genuinely leave the reachable sphere.
const UPPER_ARM := 0.30
const FOREARM := 0.28
const THIGH := 0.44
const SHIN := 0.42
const SPINE_SEG := 0.22

const BONE_COLOR := Color(0.93, 0.80, 0.62)
const JOINT_COLOR := Color(0.30, 0.72, 1.00)
const BODY_COLOR := Color(0.42, 0.47, 0.55)
const HEAD_COLOR := Color(0.93, 0.79, 0.68)

## One driven limb: a chain, its two targets, and the meshes that draw it.
class Limb:
	var chain: FabrikChain3D
	var label := ""
	var reachable := 0.0
	var target := Vector3.ZERO
	var bones: Array[MeshInstance3D] = []
	var joints: Array[MeshInstance3D] = []
	var spheres: Array[MeshInstance3D] = []
	var root := Vector3.ZERO

var elapsed := 0.0
var skeleton: Skeleton3D

var _limbs: Array[Limb] = []
var _spine: Limb
var _head: Limb
var _label: Label
var _bodies: Array[MeshInstance3D] = []
var _camera: Camera3D

func _ready() -> void:
	_build_world()
	_build_overlay()

	# Torso and head chains still solve and are covered by the body proxies, but
	# their debug bones are not drawn. The spine target is intentionally behind
	# the torso; rendering its beige debug bones through the body would look like
	# an accidental appendage on the belly.
	_spine = _make_chain("SPINE", PELVIS, [SPINE_SEG, 0.20, 0.09], 0.0, false)
	_head = _make_chain("HEAD", PELVIS + Vector3(0, 0.50, 0), [0.12, 0.10], 0.55, false)

	var shoulder_l := PELVIS + Vector3(-0.20, 0.40, 0)
	var shoulder_r := PELVIS + Vector3(0.20, 0.40, 0)
	# Smoothing is light: a heavy ease at 24 fps reads as a statue, and the
	# point here is to SEE the solve, not to admire the easing.
	var arm_l := _make_chain("ARM.L", shoulder_l, [UPPER_ARM, FOREARM], 0.25, true)
	var arm_r := _make_chain("ARM.R", shoulder_r, [UPPER_ARM, FOREARM], 0.25, true)
	var leg_l := _make_chain("LEG.L", PELVIS + Vector3(-0.10, 0.0, 0), [THIGH, SHIN], 0.5, true)
	var leg_r := _make_chain("LEG.R", PELVIS + Vector3(0.10, 0.0, 0), [THIGH, SHIN], 0.5, true)

	_limbs = [arm_l, arm_r, leg_l, leg_r]
	_build_skeleton()
	_solve_once()
	set_process(true)

func _make_chain(label: String, root: Vector3, lengths: Array, smoothing: float, draw: bool) -> Limb:
	var limb := Limb.new()
	limb.label = label
	limb.root = root
	var joints := PackedVector3Array()
	# Start as a straight downward chain; the solver keeps the root anchored and
	# bends the rest toward the target.
	for i in lengths.size() + 1:
		joints.append(root + Vector3(0.0, -float(i) * float(lengths[mini(i, lengths.size() - 1)]), 0.0))
	var seg := PackedFloat32Array()
	for length in lengths:
		seg.append(length)
	limb.chain = FabrikChain3D.new()
	limb.chain.joints = joints
	limb.chain.segment_lengths = seg
	limb.chain.root_anchored = true
	limb.chain.smoothing = smoothing
	limb.chain.tolerance = 0.00001
	limb.chain.max_iterations = 64
	limb.reachable = 0.0
	for length in lengths:
		limb.reachable += length
	if draw:
		_draw_limb(limb, lengths)
	return limb

func _draw_limb(limb: Limb, lengths: Array) -> void:
	for i in lengths.size():
		var mesh := MeshInstance3D.new()
		var radius := 0.075 if limb.label.begins_with("ARM") else 0.095
		mesh.mesh = _capsule(radius, float(lengths[i]))
		mesh.material_override = _material(BONE_COLOR)
		add_child(mesh)
		limb.bones.append(mesh)
	for i in lengths.size() + 1:
		var joint_mesh := MeshInstance3D.new()
		joint_mesh.mesh = _sphere(0.065)
		joint_mesh.material_override = _emissive(JOINT_COLOR)
		add_child(joint_mesh)
		limb.joints.append(joint_mesh)

## One body per chain root, so the figure reads as a body and not as loose sticks.
##
## The Skeleton3D carries REAL bones for the arm chain. The headless test poses
## them through pose_skeleton() and reads the rotations back, so the
## production-facing path is exercised rather than merely called.
func _build_skeleton() -> void:
	skeleton = Skeleton3D.new()
	skeleton.name = "Rig"
	add_child(skeleton)
	var arm = _limbs[0]
	var names := PackedStringArray(["upper_arm", "forearm", "hand"])
	for i in 3:
		var at: Vector3 = arm.root + Vector3(0.0, -0.30 * float(i), 0.0)
		skeleton.add_bone(names[i])
		skeleton.set_bone_rest(i, Transform3D(Basis(), at))
	_bodies.append(_body("Pelvis", PELVIS, Vector3(0.30, 0.20, 0.22), BODY_COLOR))
	_bodies.append(_body("Chest", PELVIS + Vector3(0, 0.30, 0), Vector3(0.40, 0.36, 0.24), BODY_COLOR))
	_bodies.append(_body("Head", PELVIS + Vector3(0, 0.60, 0), Vector3(0.20, 0.23, 0.21), HEAD_COLOR))

func _body(name: String, at: Vector3, size: Vector3, color: Color) -> MeshInstance3D:
	var mesh := MeshInstance3D.new()
	mesh.name = name
	var box := BoxMesh.new()
	box.size = size
	mesh.mesh = box
	mesh.position = at
	mesh.material_override = _material(color)
	add_child(mesh)
	return mesh

## Bone i spans joint i -> joint i+1. The adapter's rotation convention points a
## bone along local +Y, so the capsule is offset by half its length and then
## rotated by the derived quaternion. Nothing here recomputes the solve.
func _pose_limb(limb: Limb) -> void:
	var joints: PackedVector3Array = limb.chain.joints
	var rotations: Array = limb.chain.get_last_joint_rotations()
	for i in limb.bones.size():
		if i >= rotations.size():
			break
		var a: Vector3 = joints[i]
		var b: Vector3 = joints[i + 1]
		var bone: MeshInstance3D = limb.bones[i]
		bone.position = (a + b) * 0.5
		var q: Quaternion = rotations[i]
		# The capsule's long axis is +Y, so no extra correction is needed: this
		# is exactly the rotation the Skeleton3D path also receives.
		bone.quaternion = q
	for i in limb.joints.size():
		limb.joints[i].position = joints[i]
	var reach_marker: MeshInstance3D = limb.spheres[0] if not limb.spheres.is_empty() else null
	if reach_marker != null:
		reach_marker.visible = true

func _build_world() -> void:
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-52.0, -38.0, 0.0)
	light.light_energy = 1.15
	add_child(light)

	var fill := DirectionalLight3D.new()
	fill.rotation_degrees = Vector3(28.0, 140.0, 0.0)
	fill.light_energy = 0.45
	add_child(fill)

	# Frame the whole figure with room around it: a tight crop reads as a blob.
	_camera = Camera3D.new()
	_camera.fov = 55.0
	add_child(_camera)
	_update_camera()

	var floor_mesh := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(14.0, 14.0)
	floor_mesh.mesh = plane
	floor_mesh.material_override = _material(Color(0.10, 0.11, 0.13))
	add_child(floor_mesh)

	# Reach rings make "out of reach" visible instead of something the viewer has
	# to infer from a straightened arm. One ring per arm, at shoulder height.
	for side in [-1.0, 1.0]:
		var ring := MeshInstance3D.new()
		var torus := TorusMesh.new()
		torus.inner_radius = UPPER_ARM + FOREARM - 0.01
		torus.outer_radius = UPPER_ARM + FOREARM
		torus.rings = 64
		torus.ring_segments = 8
		ring.mesh = torus
		ring.material_override = _emissive(Color(0.25, 0.85, 0.55, 0.75))
		ring.position = PELVIS + Vector3(0.20 * side, 0.40, 0.0)
		add_child(ring)

func _build_overlay() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	_label = Label.new()
	_label.position = Vector2(14, 10)
	_label.add_theme_font_size_override("font_size", 15)
	_label.add_theme_color_override("font_color", Color(0.92, 0.94, 0.98))
	_label.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.85))
	_label.add_theme_constant_override("outline_size", 5)
	layer.add_child(_label)

func _material(color: Color) -> StandardMaterial3D:
	var mat := StandardMaterial3D.new()
	mat.albedo_color = color
	mat.roughness = 0.75
	return mat

func _emissive(color: Color) -> StandardMaterial3D:
	var mat := _material(color)
	mat.emission_enabled = true
	mat.emission = color
	mat.emission_energy_multiplier = 0.9
	return mat

func _capsule(radius: float, height: float) -> CapsuleMesh:
	var mesh := CapsuleMesh.new()
	mesh.radius = radius
	mesh.height = height
	mesh.radial_segments = 12
	mesh.rings = 4
	return mesh

func _sphere(radius: float) -> SphereMesh:
	var mesh := SphereMesh.new()
	mesh.radius = radius
	mesh.height = radius * 2.0
	mesh.radial_segments = 16
	mesh.rings = 8
	return mesh

func _process(delta: float) -> void:
	elapsed += delta
	_drive_targets()
	_solve_once()
	_update_camera()
	_update_overlay()

## Targets sweep FAR past the reach boundary. The first version of this demo
## moved them by a few centimetres, which rendered as a figure standing still -
## the arms never left a 0.58 m sphere, so there was nothing to see. Amplitudes
## here are deliberately several times the reach, so the body visibly lunges,
## over-reaches and recovers.
func _drive_targets() -> void:
	var t := elapsed

	# Torso leans towards a look target that stays INSIDE the spine's 0.51 m
	# reach. The first version aimed ~0.86 m away, so the spine reported
	# UNREACHABLE on 120/120 frames: the torso was permanently locked pointing at
	# a target it could never touch, and the figure read as a mannequin.
	_spine.chain.target = Vector3(sin(t * 0.6) * 0.22, 1.22 + sin(t * 0.45) * 0.18, -0.28)
	_head.chain.target = Vector3(sin(t * 0.6) * 0.09, 1.54 + sin(t * 0.8) * 0.08, -0.13)

	# Grips ride a radius that oscillates ACROSS the 0.58 m arm reach, so the arms
	# alternate between reaching and over-reaching. Pinning the target inside the
	# reach sphere would make the demo exercise only the easy case; swinging it
	# well outside would leave the arms permanently straight.
	var shoulder_mid := PELVIS + Vector3(0.0, 0.40, 0.0)
	var radius := 0.46 + 0.26 * sin(t * 0.8)
	var direction := Vector3(sin(t * 0.5) * 0.45, -0.12 + sin(t * 0.9) * 0.30, -1.0).normalized()
	var grip := shoulder_mid + direction * radius
	var arm_l: Limb = _limbs[0]
	var arm_r: Limb = _limbs[1]
	arm_l.chain.target = grip + Vector3(-0.22, 0.0, 0.0)
	arm_r.chain.target = grip + Vector3(0.22, 0.0, 0.0)
	arm_l.target = arm_l.chain.target
	arm_r.target = arm_r.chain.target

	# Stride: feet travel forward and back across a real step length, and lift.
	var stride := sin(t * 1.1)
	var leg_l: Limb = _limbs[2]
	var leg_r: Limb = _limbs[3]
	leg_l.chain.target = Vector3(-0.16, maxf(0.02, 0.14 + stride * 0.26), 0.30 * stride)
	leg_r.chain.target = Vector3(0.16, maxf(0.02, 0.14 - stride * 0.26), -0.30 * stride)
	leg_l.target = leg_l.chain.target
	leg_r.target = leg_r.chain.target

	_pose_limb(_spine)
	_pose_limb(_head)
	for limb in _limbs:
		_pose_limb(limb)

func _solve_once() -> void:
	for limb: Limb in [_spine, _head] + _limbs:
		limb.chain.solve()

## Slowly orbit the camera around the figure so the movie shows the solver from
## more than one angle. The camera is deterministic: elapsed is advanced by the
## render runner at exactly 24 FPS, so repeated renders produce the same path.
func _update_camera() -> void:
	if _camera == null:
		return
	var angle := elapsed * 0.35
	var focus := Vector3(0.0, 0.95, 0.0)
	_camera.position = focus + Vector3(sin(angle) * 3.6, 0.50, cos(angle) * 3.6)
	_camera.look_at(focus, Vector3.UP)

func _update_overlay() -> void:
	var lines := PackedStringArray()
	lines.append("FABRIK humanoid - 6 chains, targets crossing the reach limit")
	lines.append("")
	lines.append(_row("SPINE", _spine))
	lines.append(_row("HEAD", _head))
	for limb in _limbs:
		lines.append(_row(limb.label, limb))
	lines.append("")
	lines.append("green ring = arm reach boundary (0.58 m); segments never stretch")
	_label.text = "\n".join(lines)

func _row(label: String, limb: Limb) -> String:
	var status := limb.chain.get_last_status_name()
	var residual := limb.chain.get_last_residual()
	var distance := limb.root.distance_to(limb.target)
	var reach := limb.reachable
	var verdict := "in reach"
	if distance > reach:
		verdict = "OUT OF REACH by %.3f m" % (distance - reach)
	return "%-7s %-11s residual=%.5f  target %.2f m / reach %.2f m  %s" % [
		label, status, residual, distance, reach, verdict]
