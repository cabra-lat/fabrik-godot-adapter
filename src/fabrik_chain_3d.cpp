#include "fabrik_chain_3d.h"

#include "fabrik_core.h"
#include <godot_cpp/core/class_db.hpp>

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

int32_t FabrikChain3D::solve() {
    const int32_t count = joints.size();
    if (count < 2) {
        last_status = FABRIK_INVALID_ARGUMENT;
        last_residual = 0.0f;
        return last_status;
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
        joints.set(i, Vector3(
            coordinates[static_cast<size_t>(i) * 3U],
            coordinates[static_cast<size_t>(i) * 3U + 1U],
            coordinates[static_cast<size_t>(i) * 3U + 2U]));
    }
    return last_status;
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
    ClassDB::bind_method(D_METHOD("solve"), &FabrikChain3D::solve);
    ClassDB::bind_method(D_METHOD("get_last_status"), &FabrikChain3D::get_last_status);
    ClassDB::bind_method(D_METHOD("get_last_residual"), &FabrikChain3D::get_last_residual);
    ClassDB::bind_method(D_METHOD("get_last_status_name"), &FabrikChain3D::get_last_status_name);

    ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR3_ARRAY, "joints"), "set_joints", "get_joints");
    ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "segment_lengths"), "set_segment_lengths", "get_segment_lengths");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "target"), "set_target", "get_target");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "root_anchored"), "set_root_anchored", "get_root_anchored");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tolerance"), "set_tolerance", "get_tolerance");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "max_iterations", PROPERTY_HINT_RANGE, "1,256,1"), "set_max_iterations", "get_max_iterations");
}
