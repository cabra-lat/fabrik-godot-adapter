#include "fabrik_chain_3d.h"

#include "fabrik_core.h"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/basis.hpp>

#include <cmath>
#include <vector>

using namespace godot;

void FabrikChain3D::set_joints(const PackedVector3Array &p_joints) {
    joints = p_joints;
}

PackedVector3Array FabrikChain3D::get_joints() const {
    return joints;
}

void FabrikChain3D::set_segment_lengths(const PackedFloat32Array &p_lengths) {
    segment_lengths = p_lengths;
}

PackedFloat32Array FabrikChain3D::get_segment_lengths() const {
    return segment_lengths;
}

void FabrikChain3D::set_target(const Vector3 &p_target) {
    target = p_target;
}

Vector3 FabrikChain3D::get_target() const {
    return target;
}

void FabrikChain3D::set_pole_target(const Vector3 &p_pole_target) {
    pole_target = p_pole_target;
}

Vector3 FabrikChain3D::get_pole_target() const {
    return pole_target;
}

void FabrikChain3D::set_root_anchored(bool p_anchored) {
    root_anchored = p_anchored;
}

bool FabrikChain3D::get_root_anchored() const {
    return root_anchored;
}

void FabrikChain3D::set_tolerance(float p_tolerance) {
    tolerance = p_tolerance;
}

float FabrikChain3D::get_tolerance() const {
    return tolerance;
}

void FabrikChain3D::set_max_iterations(int32_t p_iterations) {
    max_iterations = p_iterations;
}

int32_t FabrikChain3D::get_max_iterations() const {
    return max_iterations;
}

void FabrikChain3D::set_smoothing(float p_smoothing) {
    smoothing = CLAMP(p_smoothing, 0.0f, 1.0f);
}

float FabrikChain3D::get_smoothing() const {
    return smoothing;
}

int32_t FabrikChain3D::solve() {
    const int32_t count = joints.size();
    if (count < 2) {
        last_status = FABRIK_INVALID_ARGUMENT;
        last_residual = 0.0f;
        return last_status;
    }

    // Snapshot the previous ORIENTATIONS for smoothing.
    //
    // This is a std::vector on purpose: a TypedArray copy in godot-cpp is a
    // reference to the same underlying array (_ref), not a deep copy, so
    // snapshotting into one and then writing last_rotations would overwrite the
    // snapshot too, and the slerp would compare each rotation with itself.
    std::vector<Quaternion> previous_rotations;
    previous_rotations.reserve(static_cast<size_t>(count));
    for (int32_t i = 0; i < last_rotations.size(); ++i) {
        // TypedArray::operator[] hands back a Variant, so read it into a typed
        // local before calling members on it.
        const Quaternion previous_rotation = last_rotations[i];
        previous_rotations.push_back(previous_rotation);
    }

    std::vector<float> coordinates;
    coordinates.resize(static_cast<size_t>(count) * 3U);
    for (int32_t i = 0; i < count; ++i) {
        const Vector3 point = joints[i];
        coordinates[static_cast<size_t>(i) * 3U] = point.x;
        coordinates[static_cast<size_t>(i) * 3U + 1U] = point.y;
        coordinates[static_cast<size_t>(i) * 3U + 2U] = point.z;
    }

    std::vector<float> length_values;
    const float *length_data = nullptr;
    if (segment_lengths.size() == count - 1) {
        length_values.resize(static_cast<size_t>(count - 1));
        for (int32_t i = 0; i < count - 1; ++i) {
            length_values[static_cast<size_t>(i)] = segment_lengths[i];
        }
        length_data = length_values.data();
    }
    const float target_data[3] = {target.x, target.y, target.z};
    last_status = fabrik_solve_f32(
        coordinates.data(), count, length_data, target_data,
        root_anchored ? 1 : 0, tolerance, max_iterations,
        coordinates.data(), &last_residual);
    for (int32_t i = 0; i < count; ++i) {
        const Vector3 solved(
            coordinates[static_cast<size_t>(i) * 3U],
            coordinates[static_cast<size_t>(i) * 3U + 1U],
            coordinates[static_cast<size_t>(i) * 3U + 2U]);
        joints.set(i, solved);
    }
    // The core is a position solver: it fixes the chain's reach and lengths but
    // leaves the bend plane free. Apply the optional pole hint before deriving
    // orientations, so both the raw solve and smoothing see the corrected pose.
    if (last_status != FABRIK_INVALID_ARGUMENT && last_status != FABRIK_DEGENERATE_CHAIN) {
        _apply_pole_constraint();
    }
    // Angle limits run after the pole hint. A pole target is a rigid rotation
    // about the root-to-tip axis, so it cannot change any interior angle; the two
    // constraints therefore do not fight, and running limits last means the
    // reported residual belongs to the pose that was actually kept.
    if (last_status != FABRIK_INVALID_ARGUMENT && last_status != FABRIK_DEGENERATE_CHAIN) {
        _apply_joint_limits();
        // The core measured the residual of the raw solve, before the pole and
        // limit projections moved joints. Re-measure it, or a limited chain
        // would report "solved" while visibly missing its target.
        _recompute_residual();
    }
    _update_rotations();

    // Smoothing happens in ROTATION space, never in position space.
    //
    // Interpolating joint positions towards the solved ones looks right for one
    // frame and is wrong in a way that matters: it changes every segment length,
    // so the rig's bones stretch and the visual skeleton stops matching the
    // declared one. The humanoid demo caught exactly this - a 0.30 m upper arm
    // rendered as 0.264 m while the chain eased. Slerping the orientations and
    // rebuilding the positions by forward kinematics with the declared lengths
    // cannot stretch a bone, and it converges to the same pose as the raw solve
    // because slerp(previous, raw, 1.0) IS the raw rotation.
    if (smoothing > 0.0f && previous_rotations.size() == static_cast<size_t>(count) && count >= 2) {
        const float t = 1.0f - smoothing;
        for (int32_t i = 0; i < count; ++i) {
            const Quaternion target_rotation = last_rotations[i];
            const Quaternion eased = previous_rotations[static_cast<size_t>(i)].slerp(target_rotation, t);
            last_rotations.set(i, eased);
        }
        // Forward kinematics from the current root: the root never moves here,
        // so a free-root solve eases its body rather than its anchor.
        PackedFloat32Array lengths = segment_lengths;
        if (lengths.size() != count - 1) {
            // No explicit lengths: take them from the raw solve, which is what
            // the core would have used.
            lengths.resize(count - 1);
            for (int32_t i = 0; i < count - 1; ++i) {
                lengths.set(i, joints[i].distance_to(joints[i + 1]));
            }
        }
        for (int32_t i = 0; i < count - 1; ++i) {
            const float length = lengths[i];
            if (length <= CMP_EPSILON) {
                continue;
            }
            // The bone convention is local +Y, the same one pose_skeleton uses.
            const Quaternion rotation = last_rotations[i];
            const Vector3 direction = rotation.xform(Vector3(0, 1, 0));
            joints.set(i + 1, joints[i] + direction * length);
        }
        _update_rotations();
    }
    if (last_status == FABRIK_OK) {
        emit_signal("solve_finished", last_status, last_residual);
    }
    return last_status;
}

void FabrikChain3D::set_joint_limits(const PackedVector2Array &p_limits) {
    joint_limits = p_limits;
}

PackedVector2Array FabrikChain3D::get_joint_limits() const {
    return joint_limits;
}

void FabrikChain3D::clear_joint_limits() {
    joint_limits = PackedVector2Array();
}

void FabrikChain3D::set_limit_iterations(int32_t p_iterations) {
    limit_iterations = MAX(1, p_iterations);
}

int32_t FabrikChain3D::get_limit_iterations() const {
    return limit_iterations;
}

PackedFloat32Array FabrikChain3D::get_joint_angles() const {
    const int32_t count = joints.size();
    PackedFloat32Array angles;
    angles.resize(count);
    // The ends have no interior angle, so they report 0 rather than a
    // meaningless 180 that a caller would have to special-case.
    for (int32_t i = 0; i < count; ++i) {
        angles.set(i, _interior_angle_degrees(i));
    }
    return angles;
}

int32_t FabrikChain3D::get_limit_projection_count() const {
    return last_limit_projections;
}

int32_t FabrikChain3D::get_limit_violation_count() const {
    return last_limit_violations;
}

float FabrikChain3D::_interior_angle_degrees(int32_t p_index) const {
    const int32_t count = joints.size();
    if (p_index < 1 || p_index > count - 2) {
        return 0.0f;
    }
    const Vector3 incoming = (joints[p_index] - joints[p_index - 1]);
    const Vector3 outgoing = (joints[p_index + 1] - joints[p_index]);
    if (incoming.length_squared() <= CMP_EPSILON || outgoing.length_squared() <= CMP_EPSILON) {
        return 0.0f;
    }
    const float cosine = CLAMP(incoming.normalized().dot(outgoing.normalized()), -1.0f, 1.0f);
    return Math::rad_to_deg(Math::acos(cosine));
}

Vector3 FabrikChain3D::_perpendicular_to(const Vector3 &p_direction) const {
    // Cross with the least aligned basis axis: the result is then as long as
    // possible, and it is a pure function of the direction, so two runs of the
    // same input bend the same way.
    const Vector3 basis = (Math::abs(p_direction.x) <= Math::abs(p_direction.y) &&
                                  Math::abs(p_direction.x) <= Math::abs(p_direction.z))
            ? Vector3(1, 0, 0)
            : (Math::abs(p_direction.y) <= Math::abs(p_direction.z) ? Vector3(0, 1, 0) : Vector3(0, 0, 1));
    Vector3 result = basis.cross(p_direction);
    const float length = result.length();
    if (length <= CMP_EPSILON) {
        return Vector3(0, 0, 1);
    }
    return result / length;
}

void FabrikChain3D::_apply_joint_limits() {
    last_limit_projections = 0;
    last_limit_violations = 0;
    const int32_t count = joints.size();
    if (joint_limits.is_empty() || count < 3) {
        return;
    }
    const int32_t passes = MAX(1, limit_iterations);
    for (int32_t pass = 0; pass < passes; ++pass) {
        bool projected = false;
        for (int32_t i = 1; i < count - 1; ++i) {
            // A shorter limit array leaves the remaining joints unlimited.
            if (i >= joint_limits.size()) {
                break;
            }
            const Vector2 limit = joint_limits[i];
            if (limit.x >= limit.y) {
                continue;
            }
            const float minimum = CLAMP(limit.x, 0.0f, 180.0f);
            const float maximum = CLAMP(limit.y, 0.0f, 180.0f);
            const float theta = _interior_angle_degrees(i);
            const float clamped = CLAMP(theta, minimum, maximum);
            if (Math::abs(clamped - theta) <= 0.01f) {
                continue;
            }
            const Vector3 pivot = joints[i];
            const Vector3 incoming = (pivot - joints[i - 1]);
            const Vector3 outgoing = (joints[i + 1] - pivot);
            if (incoming.length_squared() <= CMP_EPSILON || outgoing.length_squared() <= CMP_EPSILON) {
                continue;
            }
            const Vector3 in_dir = incoming.normalized();
            const Vector3 out_dir = outgoing.normalized();

            // The side of the chain the bend currently lies on decides which way
            // the corrected chain folds. A perfectly straight or perfectly folded
            // joint has no such side, so pick a stable perpendicular instead -
            // without it, a straight chain could never be given any maximum below
            // 180 degrees.
            Vector3 side = out_dir - in_dir * out_dir.dot(in_dir);
            if (side.length_squared() <= CMP_EPSILON) {
                side = _perpendicular_to(in_dir);
            } else {
                side = side.normalized();
            }
            const float radians = Math::deg_to_rad(clamped);
            const Vector3 desired =
                (in_dir * Math::cos(radians) + side * Math::sin(radians)).normalized();
            // Rotating the sub-chain BELOW the joint, rather than the one above,
            // keeps joints[0]..joints[i] - and therefore an anchored root - exactly
            // where they were, and keeps every segment length, because the whole
            // sub-chain moves rigidly. The tip is what gives way.
            const Quaternion correction(out_dir, desired);
            for (int32_t j = i + 1; j < count; ++j) {
                joints.set(j, pivot + correction.xform(joints[j] - pivot));
            }
            last_limit_projections += 1;
            projected = true;
        }
        if (!projected) {
            break;
        }
    }

    // Report what the bounded relaxation could not satisfy instead of implying
    // the limits always hold.
    for (int32_t i = 1; i < count - 1; ++i) {
        if (i >= joint_limits.size()) {
            break;
        }
        const Vector2 limit = joint_limits[i];
        if (limit.x >= limit.y) {
            continue;
        }
        const float theta = _interior_angle_degrees(i);
        if (theta < CLAMP(limit.x, 0.0f, 180.0f) - 0.01f ||
                theta > CLAMP(limit.y, 0.0f, 180.0f) + 0.01f) {
            last_limit_violations += 1;
        }
    }
}

void FabrikChain3D::_recompute_residual() {
    const int32_t count = joints.size();
    if (count < 2) {
        last_residual = 0.0f;
        return;
    }
    last_residual = joints[count - 1].distance_to(target);
}

void FabrikChain3D::_apply_pole_constraint() {
    const int32_t count = joints.size();
    if (count < 3 || pole_target.length_squared() <= CMP_EPSILON) {
        return;
    }

    const Vector3 root = joints[0];
    const Vector3 axis_full = joints[count - 1] - root;
    const float axis_length_squared = axis_full.length_squared();
    if (axis_length_squared <= CMP_EPSILON) {
        return;
    }
    const Vector3 axis = axis_full.normalized();

    // Keep only the component of the pole perpendicular to the root-tip axis:
    // that is the side the bend should face, and it is the only component a
    // rotation around the chain axis can change.
    Vector3 desired = pole_target - root;
    desired -= axis * desired.dot(axis);
    if (desired.length_squared() <= CMP_EPSILON) {
        return;
    }
    desired = desired.normalized();

    // Pick the intermediate joint furthest from the axis to define the current
    // bend side. A joint exactly on the axis gives no usable plane and is skipped.
    Vector3 current;
    float best_length_squared = CMP_EPSILON;
    for (int32_t i = 1; i < count - 1; ++i) {
        Vector3 offset = joints[i] - root;
        offset -= axis * offset.dot(axis);
        const float length_squared = offset.length_squared();
        if (length_squared > best_length_squared) {
            best_length_squared = length_squared;
            current = offset;
        }
    }
    if (current.length_squared() <= CMP_EPSILON) {
        return;
    }
    current = current.normalized();

    // Signed angle from the current bend side to the requested pole, around the
    // root-to-tip axis. std::atan2 keeps the sign instead of choosing the short
    // unsigned angle, which matters for a pole behind the chain.
    const float sine = axis.dot(current.cross(desired));
    const float cosine = current.dot(desired);
    const float angle = std::atan2(sine, cosine);
    if (std::abs(angle) <= CMP_EPSILON) {
        return;
    }

    const Quaternion turn(axis, angle);
    for (int32_t i = 1; i < count - 1; ++i) {
        joints.set(i, root + turn.xform(joints[i] - root));
    }
}

void FabrikChain3D::_update_rotations() const {
    const int32_t count = joints.size();
    last_rotations.resize(count);
    if (count < 2) {
        return;
    }

    // Bone direction: joint i spans towards joint i+1; the last joint keeps the
    // direction of the final segment.
    PackedVector3Array directions;
    directions.resize(count);
    for (int32_t i = 0; i < count - 1; ++i) {
        const Vector3 delta = joints[i + 1] - joints[i];
        directions.set(i, delta.length_squared() > CMP_EPSILON ? delta.normalized() : Vector3(0, 1, 0));
    }
    directions.set(count - 1, directions[count - 2]);

    // Seed the frame with a reference axis that is not parallel to the first
    // bone, then parallel transport it so bending never flips a bone.
    Vector3 reference(0, 1, 0);
    if (std::abs(directions[0].dot(reference)) > 0.99f) {
        reference = Vector3(1, 0, 0);
    }
    Vector3 side = reference.cross(directions[0]);
    if (side.length_squared() <= CMP_EPSILON) {
        side = Vector3(1, 0, 0).cross(directions[0]);
    }
    side = side.normalized();
    Quaternion frame;

    for (int32_t i = 0; i < count; ++i) {
        const Vector3 up = directions[i];
        if (i > 0) {
            // Shortest-arc rotation from the previous bone direction to this
            // one. godot-cpp 4.4 has Quaternion(from, to) and Quaternion::xform;
            // there is no Vector3::rotation_difference and no Quaternion*Vector3.
            const Quaternion step(directions[i - 1], up);
            frame = step * frame;
            side = step.xform(side);
        }
        // Re-orthogonalise against drift accumulated by the transport.
        side = (side - up * side.dot(up));
        if (side.length_squared() <= CMP_EPSILON) {
            Vector3 fallback(1, 0, 0);
            if (std::abs(fallback.dot(up)) > 0.99f) {
                fallback = Vector3(0, 0, 1);
            }
            side = fallback.cross(up);
        }
        side = side.normalized();
        const Vector3 forward = side.cross(up).normalized();
        last_rotations.set(i, Basis(side, up, forward).get_rotation_quaternion());
    }
}

TypedArray<Quaternion> FabrikChain3D::get_joint_rotations() const {
    TypedArray<Quaternion> rotations;
    if (joints.size() < 2) {
        return rotations;
    }
    // Derive from the live joint array without disturbing the cached result.
    const_cast<FabrikChain3D *>(this)->_update_rotations();
    rotations = last_rotations;
    return rotations;
}

TypedArray<Quaternion> FabrikChain3D::get_last_joint_rotations() const {
    return last_rotations;
}

int32_t FabrikChain3D::pose_skeleton(Skeleton3D *p_skeleton, const PackedStringArray &p_bone_names) {
    if (p_skeleton == nullptr) {
        return -1;
    }
    const TypedArray<Quaternion> rotations = get_joint_rotations();
    if (p_bone_names.size() != rotations.size()) {
        return -2;
    }
    for (int32_t i = 0; i < rotations.size(); ++i) {
        const int32_t bone = p_skeleton->find_bone(p_bone_names[i]);
        if (bone < 0) {
            return -3;
        }
        p_skeleton->set_bone_pose_rotation(bone, rotations[i]);
    }
    return rotations.size();
}

int32_t FabrikChain3D::get_last_status() const {
    return last_status;
}

float FabrikChain3D::get_last_residual() const {
    return last_residual;
}

String FabrikChain3D::get_last_status_name() const {
    return String(fabrik_status_string(last_status));
}

void FabrikChain3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_joints", "joints"), &FabrikChain3D::set_joints);
    ClassDB::bind_method(D_METHOD("get_joints"), &FabrikChain3D::get_joints);
    ClassDB::bind_method(D_METHOD("set_segment_lengths", "lengths"), &FabrikChain3D::set_segment_lengths);
    ClassDB::bind_method(D_METHOD("get_segment_lengths"), &FabrikChain3D::get_segment_lengths);
    ClassDB::bind_method(D_METHOD("set_target", "target"), &FabrikChain3D::set_target);
    ClassDB::bind_method(D_METHOD("get_target"), &FabrikChain3D::get_target);
    ClassDB::bind_method(D_METHOD("set_pole_target", "pole_target"), &FabrikChain3D::set_pole_target);
    ClassDB::bind_method(D_METHOD("get_pole_target"), &FabrikChain3D::get_pole_target);
    ClassDB::bind_method(D_METHOD("set_root_anchored", "anchored"), &FabrikChain3D::set_root_anchored);
    ClassDB::bind_method(D_METHOD("get_root_anchored"), &FabrikChain3D::get_root_anchored);
    ClassDB::bind_method(D_METHOD("set_tolerance", "tolerance"), &FabrikChain3D::set_tolerance);
    ClassDB::bind_method(D_METHOD("get_tolerance"), &FabrikChain3D::get_tolerance);
    ClassDB::bind_method(D_METHOD("set_max_iterations", "iterations"), &FabrikChain3D::set_max_iterations);
    ClassDB::bind_method(D_METHOD("get_max_iterations"), &FabrikChain3D::get_max_iterations);
    ClassDB::bind_method(D_METHOD("set_smoothing", "smoothing"), &FabrikChain3D::set_smoothing);
    ClassDB::bind_method(D_METHOD("get_smoothing"), &FabrikChain3D::get_smoothing);
    ClassDB::bind_method(D_METHOD("set_joint_limits", "limits"), &FabrikChain3D::set_joint_limits);
    ClassDB::bind_method(D_METHOD("get_joint_limits"), &FabrikChain3D::get_joint_limits);
    ClassDB::bind_method(D_METHOD("clear_joint_limits"), &FabrikChain3D::clear_joint_limits);
    ClassDB::bind_method(D_METHOD("set_limit_iterations", "iterations"), &FabrikChain3D::set_limit_iterations);
    ClassDB::bind_method(D_METHOD("get_limit_iterations"), &FabrikChain3D::get_limit_iterations);
    ClassDB::bind_method(D_METHOD("get_joint_angles"), &FabrikChain3D::get_joint_angles);
    ClassDB::bind_method(D_METHOD("get_limit_projection_count"), &FabrikChain3D::get_limit_projection_count);
    ClassDB::bind_method(D_METHOD("get_limit_violation_count"), &FabrikChain3D::get_limit_violation_count);
    ClassDB::bind_method(D_METHOD("solve"), &FabrikChain3D::solve);
    ClassDB::bind_method(D_METHOD("get_last_status"), &FabrikChain3D::get_last_status);
    ClassDB::bind_method(D_METHOD("get_last_residual"), &FabrikChain3D::get_last_residual);
    ClassDB::bind_method(D_METHOD("get_last_status_name"), &FabrikChain3D::get_last_status_name);
    ClassDB::bind_method(D_METHOD("get_joint_rotations"), &FabrikChain3D::get_joint_rotations);
    ClassDB::bind_method(D_METHOD("get_last_joint_rotations"), &FabrikChain3D::get_last_joint_rotations);
    ClassDB::bind_method(D_METHOD("pose_skeleton", "skeleton", "bone_names"), &FabrikChain3D::pose_skeleton);

    ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR3_ARRAY, "joints"), "set_joints", "get_joints");
    ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "segment_lengths"), "set_segment_lengths", "get_segment_lengths");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "target"), "set_target", "get_target");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "pole_target"), "set_pole_target", "get_pole_target");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "root_anchored"), "set_root_anchored", "get_root_anchored");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tolerance"), "set_tolerance", "get_tolerance");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "max_iterations", PROPERTY_HINT_RANGE, "1,256,1"), "set_max_iterations", "get_max_iterations");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "smoothing", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_smoothing", "get_smoothing");

    ADD_SIGNAL(MethodInfo("solve_finished",
        PropertyInfo(Variant::INT, "status"),
        PropertyInfo(Variant::FLOAT, "residual")));

    BIND_CONSTANT(FABRIK_OK);
    BIND_CONSTANT(FABRIK_INVALID_ARGUMENT);
    BIND_CONSTANT(FABRIK_UNREACHABLE);
    BIND_CONSTANT(FABRIK_NOT_CONVERGED);
    BIND_CONSTANT(FABRIK_DEGENERATE_CHAIN);
}
