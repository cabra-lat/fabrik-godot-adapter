#include "fabrik_modifier_3d.h"

#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

namespace {
// A direction this short cannot produce a meaningful rotation, and dividing by
// it would be worse. Below this, the bone is treated as having no direction and
// keeps whatever rotation it already had.
constexpr real_t kMinDirection = 1e-6;
} // namespace

void FabrikModifier3D::set_iteration_count(int32_t p_count) {
    // godot-cpp has no MAX macro, so the clamp is spelled out.
    iteration_count = p_count < 1 ? 1 : p_count;
}

int32_t FabrikModifier3D::get_iteration_count() const {
    return iteration_count;
}

TypedArray<FabrikEffector> FabrikModifier3D::get_effectors() const {
    return _collect_effectors();
}

TypedArray<FabrikEffector> FabrikModifier3D::_collect_effectors() const {
    TypedArray<FabrikEffector> found;
    const int32_t child_count = get_child_count();
    for (int32_t i = 0; i < child_count; ++i) {
        Node *child = get_child(i);
        // Direct children only, matching how GodotIK collects its effectors, and
        // so a pole-target node hanging off an effector is not mistaken for one.
        FabrikEffector *effector = Object::cast_to<FabrikEffector>(child);
        if (effector != nullptr) {
            found.append(effector);
        }
    }
    return found;
}

PackedInt32Array FabrikModifier3D::_build_bone_chain(const FabrikEffector *p_effector, Skeleton3D *p_skeleton) const {
    PackedInt32Array bones;
    if (p_effector == nullptr || p_skeleton == nullptr) {
        return bones;
    }
    const int32_t bone_count = p_skeleton->get_bone_count();
    int32_t bone = p_effector->get_bone_idx();
    if (bone < 0 || bone >= bone_count) {
        return bones;
    }
    // A chain that starts at the root has nothing to bend against: the root is
    // the fixed point, so there is no second bone to move. GodotIK skips these
    // too, and skipping is better than solving a no-op every frame.
    if (p_skeleton->get_bone_parent(bone) < 0) {
        return bones;
    }
    for (int32_t step = 0; step < p_effector->get_chain_length() && bone >= 0; ++step) {
        bones.append(bone);
        bone = p_skeleton->get_bone_parent(bone);
    }
    return bones;
}

void FabrikModifier3D::_ensure_capacity(int32_t p_bone_count) {
    if (int32_t(new_global.size()) < p_bone_count) {
        new_global.resize(p_bone_count);
        snapshot_global.resize(p_bone_count);
        touched.assign(p_bone_count, false);
    }
}

Transform3D FabrikModifier3D::_current_global(int32_t p_bone, Skeleton3D *p_skeleton) const {
    if (touched[p_bone]) {
        return new_global[p_bone];
    }
    return p_skeleton->get_bone_global_pose(p_bone);
}

void FabrikModifier3D::_write_pose(int32_t p_bone, const Transform3D &p_global, Skeleton3D *p_skeleton) {
    new_global[p_bone] = p_global;
    touched[p_bone] = true;
    // Skeleton3D's own relation: a bone's pose is its global transform relative
    // to the parent's global transform. Measured, not assumed - see
    // docs/GODOTIK_COMPATIBILITY.md.
    const int32_t parent = p_skeleton->get_bone_parent(p_bone);
    if (parent < 0) {
        p_skeleton->set_bone_pose(p_bone, p_global);
        return;
    }
    p_skeleton->set_bone_pose(p_bone, _current_global(parent, p_skeleton).affine_inverse() * p_global);
}

int32_t FabrikModifier3D::_solve_chain(const FabrikEffector *p_effector, Skeleton3D *p_skeleton, const PackedInt32Array &r_bones, float &r_residual) {
    const int32_t count = r_bones.size();
    r_residual = 0.0f;
    if (count < 2) {
        return -1; // A single bone cannot bend; the core would report it as such.
    }

    // Snapshot the chain as the solver should see it: current global poses,
    // which is also where each segment's length is measured from. Measuring
    // from the pose rather than the rest means a skeleton already deformed by
    // another modifier keeps its own proportions.
    PackedVector3Array joints;
    PackedFloat32Array lengths;
    joints.resize(count);
    for (int32_t i = 0; i < count; ++i) {
        snapshot_global[r_bones[i]] = p_skeleton->get_bone_global_pose(r_bones[i]);
        joints[i] = snapshot_global[r_bones[i]].origin;
    }
    for (int32_t i = 0; i < count - 1; ++i) {
        lengths.append(joints[i].distance_to(joints[i + 1]));
    }

    // Influence blends the goal towards where the leaf is now, so a ramp from 0
    // eases the solve in and a ramp to 0 lets the skeleton fall back to
    // whatever the rest of the pipeline produced.
    const float influence = get_influence() * p_effector->get_influence();
    Vector3 goal = p_skeleton->to_local(p_effector->get_global_position());
    const Vector3 current_leaf = joints[0];
    if (influence < 1.0f) {
        goal = current_leaf.lerp(goal, influence);
    }

    Ref<FabrikChain3D> chain;
    chain.instantiate();
    chain->set_joints(joints);
    chain->set_segment_lengths(lengths);
    chain->set_target(goal);
    chain->set_root_anchored(true);
    chain->set_max_iterations(iteration_count);
    // A pole hint only works in the same space as the chain, hence to_local.
    Node3D *pole = p_effector->get_pole_target();
    if (pole != nullptr) {
        chain->set_pole_target(p_skeleton->to_local(pole->get_global_position()));
    }
    chain->solve();
    const PackedVector3Array solved = chain->get_joints();
    const int32_t status = chain->get_last_status();
    r_residual = chain->get_last_residual();

    for (int32_t i = 0; i < count; ++i) {
        const int32_t bone = r_bones[i];
        const Transform3D before = snapshot_global[bone];
        Transform3D after(before.basis, solved[i]);
        if (i + 1 < count) {
            // Rotate the bone by the smallest turn that carries its current
            // direction onto the solved one. Rotating the existing basis (rather
            // than building one from +Y) keeps the bone's own axes and its twist,
            // and needs no assumption about how the bone was authored.
            const Vector3 current_dir = before.origin.direction_to(snapshot_global[r_bones[i + 1]].origin);
            const Vector3 solved_dir = solved[i].direction_to(solved[i + 1]);
            if (current_dir.length() > kMinDirection && solved_dir.length() > kMinDirection) {
                after.basis = Basis(Quaternion(current_dir, solved_dir)) * before.basis;
            }
        }
        // The leaf is r_bones[0] - the effector bone itself - not the last entry.
        // These modes are about the bone the effector drives, which is the one
        // GodotIK means by "the leaf".
        const bool is_leaf = i == 0;
        switch (p_effector->get_transform_mode()) {
            case FabrikEffector::PRESERVE_ROTATION: {
                // Explicitly refuse any rotation change on the leaf. Same shape
                // as GodotIK's PRESERVE_ROTATION.
                if (is_leaf) {
                    after.basis = before.basis;
                }
            } break;
            case FabrikEffector::FULL_TRANSFORM: {
                if (is_leaf) {
                    after.basis = (p_skeleton->get_global_transform().affine_inverse() * p_effector->get_global_transform()).basis;
                }
            } break;
            default:
                break;
        }
        _write_pose(bone, after, p_skeleton);
    }

    // STRAIGHTEN_CHAIN is a pose-level edit: drop the leaf's own rotation, so
    // the chain's last segment continues the parent bone's direction. It has to
    // happen after the leaf has been written, because it replaces the pose.
    if (p_effector->get_transform_mode() == FabrikEffector::STRAIGHTEN_CHAIN) {
        const int32_t leaf = r_bones[0];
        Transform3D local = p_skeleton->get_bone_pose(leaf);
        local.basis = Basis().scaled(local.basis.get_scale());
        p_skeleton->set_bone_pose(leaf, local);
        // Keep the working set consistent: local is parent-relative, so undo the
        // conversion rather than storing a pose-space origin as if it were global.
        const int32_t leaf_parent = p_skeleton->get_bone_parent(leaf);
        new_global[leaf] = leaf_parent >= 0 ? _current_global(leaf_parent, p_skeleton) * local : local;
    }
    return status;
}

int32_t FabrikModifier3D::solve_now() {
    last_statuses.clear();
    last_error = String();
    last_chain_count = 0;
    last_max_residual = 0.0f;

    Skeleton3D *skeleton = Object::cast_to<Skeleton3D>(get_skeleton());
    if (skeleton == nullptr) {
        last_error = "no Skeleton3D above the modifier";
        return -1;
    }
    _ensure_capacity(skeleton->get_bone_count());
    for (int32_t i = 0; i < skeleton->get_bone_count(); ++i) {
        touched[i] = false;
    }

    const TypedArray<FabrikEffector> effectors = _collect_effectors();
    for (int32_t i = 0; i < effectors.size(); ++i) {
        // TypedArray's operator[] hands back a Variant, and a bare Variant to
        // pointer conversion is ambiguous - the cast is not optional.
        FabrikEffector *effector = Object::cast_to<FabrikEffector>(effectors[i]);
        if (effector == nullptr) {
            continue;
        }
        // Re-resolve every frame: a bone can be renamed, added, or removed
        // without any signal this class listens to.
        effector->refresh_bone_idx(skeleton);
        if (!effector->is_active() || effector->get_influence() <= 0.0f) {
            continue;
        }
        const PackedInt32Array bones = _build_bone_chain(effector, skeleton);
        if (bones.size() < 2) {
            // Reported, not silently dropped: a misconfigured effector is the
            // difference between "it does nothing" and "it is not configured".
            last_error = "effector '" + effector->get_name() + "' has no usable chain (bone " +
                    String::num_int64(effector->get_bone_idx()) + ")";
            continue;
        }
        float residual = 0.0f;
        const int32_t status = _solve_chain(effector, skeleton, bones, residual);
        last_chain_count += 1;
        last_statuses.append(status);
        if (residual > last_max_residual) {
            last_max_residual = residual;
        }
    }

    // A moving effector has to reach the mesh this frame, not next frame's.
    skeleton->force_update_bone_child_transform(0);
    return last_chain_count;
}

void FabrikModifier3D::_process_modification() {
    solve_now();
}

int32_t FabrikModifier3D::get_last_chain_count() const {
    return last_chain_count;
}

PackedInt32Array FabrikModifier3D::get_last_statuses() const {
    return last_statuses;
}

String FabrikModifier3D::get_last_error() const {
    return last_error;
}

float FabrikModifier3D::get_last_max_residual() const {
    return last_max_residual;
}

void FabrikModifier3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_iteration_count", "count"), &FabrikModifier3D::set_iteration_count);
    ClassDB::bind_method(D_METHOD("get_iteration_count"), &FabrikModifier3D::get_iteration_count);
    ClassDB::bind_method(D_METHOD("get_effectors"), &FabrikModifier3D::get_effectors);
    ClassDB::bind_method(D_METHOD("solve_now"), &FabrikModifier3D::solve_now);
    ClassDB::bind_method(D_METHOD("get_last_chain_count"), &FabrikModifier3D::get_last_chain_count);
    ClassDB::bind_method(D_METHOD("get_last_statuses"), &FabrikModifier3D::get_last_statuses);
    ClassDB::bind_method(D_METHOD("get_last_error"), &FabrikModifier3D::get_last_error);
    ClassDB::bind_method(D_METHOD("get_last_max_residual"), &FabrikModifier3D::get_last_max_residual);

    ADD_PROPERTY(PropertyInfo(Variant::INT, "iteration_count", PROPERTY_HINT_RANGE, "1,64,1"), "set_iteration_count", "get_iteration_count");
}
