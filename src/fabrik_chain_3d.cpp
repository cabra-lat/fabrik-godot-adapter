#include "fabrik_chain_3d.h"

#include "fabrik_core.h"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/basis.hpp>

#include <vector>

using namespace godot;

// Flat buffer helpers. The core's ABI is flat row-major float arrays, and these
// are the only place the adapter converts between Godot's packed arrays and
// that layout. Everything past this point is the core's arithmetic, not ours.
static void joints_to_flat(const PackedVector3Array &p_joints, std::vector<float> &r_flat) {
    const int32_t count = p_joints.size();
    r_flat.resize(static_cast<size_t>(count) * 3U);
    for (int32_t i = 0; i < count; ++i) {
        const Vector3 point = p_joints[i];
        r_flat[static_cast<size_t>(i) * 3U] = point.x;
        r_flat[static_cast<size_t>(i) * 3U + 1U] = point.y;
        r_flat[static_cast<size_t>(i) * 3U + 2U] = point.z;
    }
}

static void flat_to_joints(const std::vector<float> &p_flat, int32_t p_count, PackedVector3Array &r_joints) {
    for (int32_t i = 0; i < p_count; ++i) {
        const Vector3 solved(
            p_flat[static_cast<size_t>(i) * 3U],
            p_flat[static_cast<size_t>(i) * 3U + 1U],
            p_flat[static_cast<size_t>(i) * 3U + 2U]);
        r_joints.set(i, solved);
    }
}

// Quaternions cross as 4 floats per joint in (w, x, y, z), which is Godot's own
// component order, so this is a copy rather than a conversion. Keeping it a copy
// is deliberate: a reinterpret_cast between Vector3 and float arrays is exactly
// the kind of shortcut that survives review and breaks on a different compiler.
static void quats_to_flat(const TypedArray<Quaternion> &p_quats, std::vector<float> &r_flat) {
    const int32_t count = p_quats.size();
    r_flat.resize(static_cast<size_t>(count) * 4U);
    for (int32_t i = 0; i < count; ++i) {
        const Quaternion q = p_quats[i];
        r_flat[static_cast<size_t>(i) * 4U] = q.w;
        r_flat[static_cast<size_t>(i) * 4U + 1U] = q.x;
        r_flat[static_cast<size_t>(i) * 4U + 2U] = q.y;
        r_flat[static_cast<size_t>(i) * 4U + 3U] = q.z;
    }
}

static void flat_to_quats(const std::vector<float> &p_flat, int32_t p_count, TypedArray<Quaternion> &r_quats) {
    r_quats.resize(p_count);
    for (int32_t i = 0; i < p_count; ++i) {
        // The core's layout is (w, x, y, z) and godot-cpp's positional
        // constructor is (x, y, z, w) - NOT w first, which is what the engine
        // docs and Basis::get_rotation_quaternion() suggest. Passing them in the
        // same order silently produces the wrong rotation rather than an error,
        // so the swap is deliberate and commented.
        const Quaternion q(
            p_flat[static_cast<size_t>(i) * 4U + 1U],
            p_flat[static_cast<size_t>(i) * 4U + 2U],
            p_flat[static_cast<size_t>(i) * 4U + 3U],
            p_flat[static_cast<size_t>(i) * 4U]);
        r_quats.set(i, q);
    }
}

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
    std::vector<float> previous_quaternions;
    quats_to_flat(last_rotations, previous_quaternions);

    std::vector<float> coordinates;
    joints_to_flat(joints, coordinates);

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
    flat_to_joints(coordinates, count, joints);
    const bool solved = last_status != FABRIK_INVALID_ARGUMENT && last_status != FABRIK_DEGENERATE_CHAIN;

    // The core is a position solver: it fixes the chain's reach and lengths but
    // leaves the bend plane free. The optional pole hint, the angle limits, the
    // rotations and the smoothing all live in the core now, so this method is
    // orchestration only - it decides the ORDER and owns nothing else. The order
    // is the core's documented one, and 2 and 3 cannot fight: a pole target is a
    // rigid rotation about the root-to-tip axis and so changes no interior angle,
    // which is why running the limits afterwards is safe and means the reported
    // residual belongs to the pose that was actually kept.
    if (solved) {
        const float pole_data[3] = {pole_target.x, pole_target.y, pole_target.z};
        fabrik_apply_pole_f32(coordinates.data(), count, pole_data, coordinates.data());

        if (!joint_limits.is_empty()) {
            const int32_t limit_count = joint_limits.size();
            std::vector<float> limit_values(static_cast<size_t>(limit_count) * 2U);
            for (int32_t i = 0; i < limit_count; ++i) {
                const Vector2 limit = joint_limits[i];
                limit_values[static_cast<size_t>(i) * 2U] = limit.x;
                limit_values[static_cast<size_t>(i) * 2U + 1U] = limit.y;
            }
            fabrik_apply_joint_limits_f32(coordinates.data(), count, limit_values.data(),
                limit_count, limit_iterations, coordinates.data(),
                &last_limit_projections, &last_limit_violations);
        } else {
            last_limit_projections = 0;
            last_limit_violations = 0;
        }
        flat_to_joints(coordinates, count, joints);

        // The core measured the residual of the raw solve, before the pole and
        // limit projections moved joints. Re-measure it, or a limited chain would
        // report "solved" while visibly missing its target.
        fabrik_residual_f32(coordinates.data(), count, target_data, &last_residual);
    }

    // Rotations, and then smoothing, which is a rotation-space operation: see
    // fabrik_smooth_rotations_f32 for why interpolating positions would stretch
    // bones.
    std::vector<float> raw_quaternions;
    raw_quaternions.resize(static_cast<size_t>(count) * 4U);
    if (solved) {
        fabrik_derive_rotations_f32(coordinates.data(), count, raw_quaternions.data());
    }
    if (smoothing > 0.0f && previous_quaternions.size() == static_cast<size_t>(count) * 4U && count >= 2) {
        std::vector<float> eased_quaternions(static_cast<size_t>(count) * 4U);
        if (length_data == nullptr) {
            // No declared lengths: take them from the raw solve, which is what
            // the core itself would have used. The buffer must be sized first -
            // writing into an empty vector's data() is undefined behaviour, and
            // it would only show up as a sanitizer report on someone else's run.
            length_values.assign(static_cast<size_t>(count - 1), 0.0f);
            fabrik_measure_lengths_f32(coordinates.data(), count, length_values.data());
        }
        fabrik_smooth_rotations_f32(coordinates.data(), count, length_values.data(),
            previous_quaternions.data(), raw_quaternions.data(), smoothing,
            coordinates.data(), eased_quaternions.data());
        raw_quaternions.swap(eased_quaternions);
        flat_to_joints(coordinates, count, joints);
    }
    flat_to_quats(raw_quaternions, count, last_rotations);
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
    // The ends have no flexion angle, so they report 0 rather than a meaningless
    // 180 that a caller would have to special-case. The angle itself is the
    // core's, and it is a FLEXION angle: 0 is straight, 180 is folded back.
    std::vector<float> flat_joints;
    joints_to_flat(joints, flat_joints);
    std::vector<float> flat_angles(static_cast<size_t>(count), 0.0f);
    if (count >= 2 && fabrik_joint_angles_f32(flat_joints.data(), count, flat_angles.data()) == FABRIK_OK) {
        for (int32_t i = 0; i < count; ++i) {
            angles.set(i, flat_angles[static_cast<size_t>(i)]);
        }
    }
    return angles;
}

int32_t FabrikChain3D::get_limit_projection_count() const {
    return last_limit_projections;
}

int32_t FabrikChain3D::get_limit_violation_count() const {
    return last_limit_violations;
}


void FabrikChain3D::_update_rotations() const {
    // The parallel-transported frame, the reference-axis seeding and the basis to
    // quaternion conversion all live in the core now. What is left here is the
    // conversion between Godot's packed arrays and the core's flat layout, which
    // is engine interop rather than arithmetic.
    const int32_t count = joints.size();
    last_rotations.resize(count);
    if (count < 2) {
        return;
    }
    std::vector<float> flat_joints;
    joints_to_flat(joints, flat_joints);
    std::vector<float> flat_quaternions(static_cast<size_t>(count) * 4U, 0.0f);
    if (fabrik_derive_rotations_f32(flat_joints.data(), count, flat_quaternions.data()) != FABRIK_OK) {
        return;
    }
    flat_to_quats(flat_quaternions, count, const_cast<TypedArray<Quaternion> &>(last_rotations));
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
    ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR2_ARRAY, "joint_limits"), "set_joint_limits", "get_joint_limits");
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
