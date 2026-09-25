#include "fabrik_chain_3d.h"

#include "fabrik_core.h"
#include <godot_cpp/core/class_db.hpp>
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

    // Keep the pre-solve pose so smoothing can interpolate towards the new one.
    const PackedVector3Array previous = joints;

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
        // smoothing == 0 keeps the raw solve; 1 freezes the chain.
        joints.set(i, smoothing > 0.0f ? previous[i].lerp(solved, 1.0f - smoothing) : solved);
    }
    _update_rotations();
    if (last_status == FABRIK_OK) {
        emit_signal("solve_finished", last_status, last_residual);
    }
    return last_status;
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
    ClassDB::bind_method(D_METHOD("set_root_anchored", "anchored"), &FabrikChain3D::set_root_anchored);
    ClassDB::bind_method(D_METHOD("get_root_anchored"), &FabrikChain3D::get_root_anchored);
    ClassDB::bind_method(D_METHOD("set_tolerance", "tolerance"), &FabrikChain3D::set_tolerance);
    ClassDB::bind_method(D_METHOD("get_tolerance"), &FabrikChain3D::get_tolerance);
    ClassDB::bind_method(D_METHOD("set_max_iterations", "iterations"), &FabrikChain3D::set_max_iterations);
    ClassDB::bind_method(D_METHOD("get_max_iterations"), &FabrikChain3D::get_max_iterations);
    ClassDB::bind_method(D_METHOD("set_smoothing", "smoothing"), &FabrikChain3D::set_smoothing);
    ClassDB::bind_method(D_METHOD("get_smoothing"), &FabrikChain3D::get_smoothing);
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
