#include "fabrik_effector.h"

#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void FabrikEffector::set_bone_name(const String &p_name) {
    bone_name = p_name;
    if (bone_name.is_empty()) {
        bone_idx = -1;
        return;
    }
    // A name without a skeleton behind it cannot be resolved yet; the modifier
    // re-resolves every solve, so leave bone_idx alone rather than guessing.
    Skeleton3D *skeleton = Object::cast_to<Skeleton3D>(get_parent());
    if (skeleton == nullptr && get_parent() != nullptr) {
        skeleton = Object::cast_to<Skeleton3D>(get_parent()->get_parent());
    }
    if (skeleton != nullptr) {
        bone_idx = skeleton->find_bone(bone_name);
    }
}

void FabrikEffector::refresh_bone_idx(Skeleton3D *p_skeleton) {
    if (bone_name.is_empty()) {
        return;
    }
    if (p_skeleton == nullptr) {
        bone_idx = -1;
        return;
    }
    bone_idx = p_skeleton->find_bone(bone_name);
}

String FabrikEffector::get_bone_name() const {
    return bone_name;
}

void FabrikEffector::set_bone_idx(int32_t p_bone_idx) {
    bone_idx = p_bone_idx;
    // Keep the name and the index telling the same story: a scene saved with an
    // index and reopened with a different bone list must not silently disagree.
    Skeleton3D *skeleton = nullptr;
    if (get_parent() != nullptr) {
        skeleton = Object::cast_to<Skeleton3D>(get_parent()->get_parent());
    }
    if (skeleton != nullptr && p_bone_idx >= 0 && p_bone_idx < skeleton->get_bone_count()) {
        bone_name = skeleton->get_bone_name(p_bone_idx);
    }
}

int32_t FabrikEffector::get_bone_idx() const {
    return bone_idx;
}

void FabrikEffector::set_chain_length(int32_t p_chain_length) {
    // godot-cpp has no MAX/CLAMP macros, so the clamps are spelled out.
    chain_length = p_chain_length < 1 ? 1 : p_chain_length;
}

int32_t FabrikEffector::get_chain_length() const {
    return chain_length;
}

void FabrikEffector::set_active(bool p_active) {
    active = p_active;
}

bool FabrikEffector::is_active() const {
    return active;
}

void FabrikEffector::set_influence(float p_influence) {
    influence = p_influence < 0.0f ? 0.0f : (p_influence > 1.0f ? 1.0f : p_influence);
}

float FabrikEffector::get_influence() const {
    return influence;
}

void FabrikEffector::set_transform_mode(TransformMode p_transform_mode) {
    transform_mode = p_transform_mode;
}

FabrikEffector::TransformMode FabrikEffector::get_transform_mode() const {
    return transform_mode;
}

void FabrikEffector::set_pole_target_path(const NodePath &p_path) {
    pole_target_path = p_path;
}

NodePath FabrikEffector::get_pole_target_path() const {
    return pole_target_path;
}

Node3D *FabrikEffector::get_pole_target() const {
    if (pole_target_path.is_empty()) {
        return nullptr;
    }
    return Object::cast_to<Node3D>(get_node_or_null(pole_target_path));
}

void FabrikEffector::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_bone_name", "name"), &FabrikEffector::set_bone_name);
    ClassDB::bind_method(D_METHOD("get_bone_name"), &FabrikEffector::get_bone_name);
    ClassDB::bind_method(D_METHOD("set_bone_idx", "bone_idx"), &FabrikEffector::set_bone_idx);
    ClassDB::bind_method(D_METHOD("get_bone_idx"), &FabrikEffector::get_bone_idx);
    ClassDB::bind_method(D_METHOD("set_chain_length", "length"), &FabrikEffector::set_chain_length);
    ClassDB::bind_method(D_METHOD("get_chain_length"), &FabrikEffector::get_chain_length);
    ClassDB::bind_method(D_METHOD("set_active", "active"), &FabrikEffector::set_active);
    ClassDB::bind_method(D_METHOD("is_active"), &FabrikEffector::is_active);
    ClassDB::bind_method(D_METHOD("set_influence", "influence"), &FabrikEffector::set_influence);
    ClassDB::bind_method(D_METHOD("get_influence"), &FabrikEffector::get_influence);
    ClassDB::bind_method(D_METHOD("set_transform_mode", "mode"), &FabrikEffector::set_transform_mode);
    ClassDB::bind_method(D_METHOD("get_transform_mode"), &FabrikEffector::get_transform_mode);
    ClassDB::bind_method(D_METHOD("set_pole_target_path", "path"), &FabrikEffector::set_pole_target_path);
    ClassDB::bind_method(D_METHOD("get_pole_target_path"), &FabrikEffector::get_pole_target_path);
    ClassDB::bind_method(D_METHOD("get_pole_target"), &FabrikEffector::get_pole_target);

    ADD_PROPERTY(PropertyInfo(Variant::STRING, "bone_name"), "set_bone_name", "get_bone_name");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "chain_length", PROPERTY_HINT_RANGE, "1,32,1"), "set_chain_length", "get_chain_length");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "active"), "set_active", "is_active");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "influence", PROPERTY_HINT_RANGE, "0,1,0.001"), "set_influence", "get_influence");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "transform_mode", PROPERTY_HINT_ENUM, "Position Only,Preserve Rotation,Straighten Chain,Full Transform"), "set_transform_mode", "get_transform_mode");
    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "pole_target_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), "set_pole_target_path", "get_pole_target_path");

    BIND_ENUM_CONSTANT(POSITION_ONLY);
    BIND_ENUM_CONSTANT(PRESERVE_ROTATION);
    BIND_ENUM_CONSTANT(STRAIGHTEN_CHAIN);
    BIND_ENUM_CONSTANT(FULL_TRANSFORM);
}
