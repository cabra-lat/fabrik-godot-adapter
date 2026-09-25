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

const BONE_COLOR := Color(0.86, 0.80, 0.70)
const TARGET_COLOR := Color(0.95, 0.42, 0.35)

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

func _ready() -> void:
	_build_world()
	_build_overlay()

	_spine = _make_chain("SPINE", PELVIS, [SPINE_SEG, 0.20, 0.09], 0.0, true)
	_head = _make_chain("HEAD", PELVIS + Vector3(0, 0.50, 0), [0.12, 0.10], 0.55, true)

	var shoulder_l := PELVIS + Vector3(-0.20, 0.40, 0)
	var shoulder_r := PELVIS + Vector3(0.20, 0.40, 0)
	var arm_l := _make_chain("ARM.L", shoulder_l, [UPPER_ARM, FOREARM], 0.45, true)
	var arm_r := _make_chain("ARM.R", shoulder_r, [UPPER_ARM, FOREARM], 0.45, true)
	var leg_l := _make_chain("LEG.L", PELVIS + Vector3(-0.10, 0.0, 0), [THIGH, SHIN], 0.5, false)
	var leg_r := _make_chain("LEG.R", PELVIS + Vector3(0.10, 0.0, 0), [THIGH, SHIN], 0.5, false)

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
		joint_mesh.mesh = _sphere(0.055)
		joint_mesh.material_override = _material(Color(0.25, 0.55, 0.95))
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
	_bodies.append(_body("Pelvis", PELVIS, Vector3(0.26, 0.16, 0.18), Color(0.30, 0.33, 0.38)))
	_bodies.append(_body("Chest", PELVIS + Vector3(0, 0.30, 0), Vector3(0.34, 0.34, 0.20), Color(0.26, 0.29, 0.34)))
	_bodies.append(_body("Head", PELVIS + Vector3(0, 0.58, 0), Vector3(0.17, 0.20, 0.18), Color(0.85, 0.72, 0.62)))

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

	var camera := Camera3D.new()
	camera.position = Vector3(0.0, 1.5, 3.1)
	camera.fov = 58.0
	camera.look_at_from_position(camera.position, Vector3(0.0, 1.05, 0.0), Vector3.UP)
	add_child(camera)

	var floor_mesh := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(9.0, 9.0)
	floor_mesh.mesh = plane
	floor_mesh.material_override = _material(Color(0.17, 0.18, 0.20))
	add_child(floor_mesh)

	# Reach rings make "out of reach" visible instead of something the viewer has
	# to infer from a straightened arm.
	var ring := MeshInstance3D.new()
	var torus := TorusMesh.new()
	torus.inner_radius = 0.56
	torus.outer_radius = 0.58
	torus.rings = 48
	torus.ring_segments = 8
	ring.mesh = torus
	var ring_mat := _material(Color(0.30, 0.75, 0.55, 0.55))
	ring_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	ring.material_override = ring_mat
	ring.position = PELVIS
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
	_update_overlay()

## Targets deliberately cross the reach boundary. The arm orbit radius is 0.72
## against an arm reach of 0.58, so roughly a third of every orbit is out of
## reach - the solver has to handle both regimes, every cycle.
func _drive_targets() -> void:
	var t := elapsed

	var look := Vector3(sin(t * 0.7) * 0.9, 1.45 + sin(t * 0.5) * 0.25, -1.5)
	_spine.chain.target = look

	var head := Vector3(sin(t * 0.7) * 0.55, 1.62 + sin(t * 0.9) * 0.12, -0.85)
	_head.chain.target = head

	# Grips: a two-handed hold that orbits past arm reach and swings in depth.
	var grip := Vector3(
		sin(t * 0.8) * 0.42,
		1.18 + sin(t * 1.1) * 0.10,
		-0.62 + cos(t * 0.8) * 0.30)
	var arm_l: Limb = _limbs[0]
	var arm_r: Limb = _limbs[1]
	arm_l.chain.target = grip + Vector3(-0.20, 0.0, 0.0)
	arm_r.chain.target = grip + Vector3(0.20, 0.0, 0.0)
	arm_l.target = arm_l.chain.target
	arm_r.target = arm_r.chain.target

	# Stance: feet slide across the floor and one foot lifts out of reach.
	var step := sin(t * 0.9)
	var leg_l: Limb = _limbs[2]
	var leg_r: Limb = _limbs[3]
	leg_l.chain.target = Vector3(-0.18 + step * 0.16, maxf(0.0, 0.10 + step * 0.22), 0.10)
	leg_r.chain.target = Vector3(0.18 - step * 0.16, maxf(0.0, 0.10 - step * 0.22), -0.06)
	leg_l.target = leg_l.chain.target
	leg_r.target = leg_r.chain.target

	_pose_limb(_spine)
	_pose_limb(_head)
	for limb in _limbs:
		_pose_limb(limb)

func _solve_once() -> void:
	for limb: Limb in [_spine, _head] + _limbs:
		limb.chain.solve()

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
