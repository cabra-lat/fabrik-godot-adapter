#include "fabrik_modifier_3d.h"

#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/error_macros.hpp>
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
    // Every bone index below is used to index the per-bone working set, so a
    // bone the working set does not cover is a crash, not a wrong answer. The
    // guard is cheap and it turns a segfault into a diagnosable refusal: the
    // engine can call this before the skeleton reports the bones a chain
    // references.
    if (int32_t(new_global.size()) < p_skeleton->get_bone_count() || count > p_skeleton->get_bone_count()) {
        _ensure_capacity(p_skeleton->get_bone_count());
    }
    for (int32_t i = 0; i < count; ++i) {
        if (r_bones[i] < 0 || r_bones[i] >= int32_t(new_global.size())) {
            ERR_PRINT("[FabrikModifier3D] bone " + String::num_int64(r_bones[i])
                + " is outside the working set of " + String::num_int64(new_global.size())
                + " bones; skipping the chain");
            r_residual = 0.0f;
            return -1;
        }
    }

    // r_bones is leaf-first (the effector's bone first, then its parents), but
    // the core's convention is joints[0] = root and joints[count-1] = tip.
    // Feeding it leaf-first silently solves the wrong problem: the core anchors
    // joints[0], so the TIP would be pinned and the root dragged onto the
    // target, and the reach test measures from the wrong end - which is exactly
    // how this was found, as a straight chain reported UNREACHABLE.
    PackedVector3Array joints;
    PackedFloat32Array lengths;
    joints.resize(count);
    for (int32_t i = 0; i < count; ++i) {
        const int32_t bone = r_bones[count - 1 - i];
        snapshot_global[bone] = p_skeleton->get_bone_global_pose(bone);
        joints[i] = snapshot_global[bone].origin;
    }
    for (int32_t i = 0; i < count - 1; ++i) {
        lengths.append(joints[i].distance_to(joints[i + 1]));
    }

    // Influence blends the goal towards where the tip is now, so a ramp from 0
    // eases the solve in and a ramp to 0 lets the skeleton fall back to
    // whatever the rest of the pipeline produced.
    const float influence = get_influence() * p_effector->get_influence();
    Vector3 goal = p_skeleton->to_local(p_effector->get_global_position());
    const Vector3 current_tip = joints[count - 1];
    if (influence < 1.0f) {
        goal = current_tip.lerp(goal, influence);
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

    // Fill the working set. Nothing is written to the skeleton here: a bone's
    // pose is relative to its parent's, so writing before every parent is known
    // lands each bone relative to a transform that is about to change.
    for (int32_t i = 0; i < count; ++i) {
        const int32_t bone = r_bones[i];             // leaf-first
        const int32_t solved_index = count - 1 - i;  // solved is root-first
        const Transform3D before = snapshot_global[bone];
        Transform3D after(before.basis, solved[solved_index]);
        // A bone's forward direction runs along its chain. The bone at
        // r_bones[i] sits at solved_index = count - 1 - i in the root-first
        // solved array, and its chain child - the next step TOWARDS the leaf -
        // is r_bones[i - 1], which is solved_index + 1. The leaf (i == 0) has no
        // child and takes its direction from its chain parent instead, which is
        // the last segment of the chain.
        //
        // Both directions are bounds-checked against `solved` rather than
        // reasoned about: indexing solved_index - 1 here reads solved[-1] at the
        // chain root (solved_index == 0) and segfaults, which is exactly what
        // this code did for two CI runs.
        Vector3 current_dir;
        Vector3 solved_dir;
        if (i > 0 && solved_index + 1 < solved.size()) {
            const int32_t child = r_bones[i - 1];
            current_dir = before.origin.direction_to(snapshot_global[child].origin);
            solved_dir = solved[solved_index].direction_to(solved[solved_index + 1]);
        } else if (i == 0 && count > 1 && solved_index >= 1) {
            current_dir = snapshot_global[r_bones[1]].origin.direction_to(before.origin);
            solved_dir = solved[solved_index - 1].direction_to(solved[solved_index]);
        }
        // Rotate the bone by the smallest turn that carries its current
        // direction onto the solved one. Rotating the existing basis (rather
        // than building one from +Y) keeps the bone's own axes and its twist,
        // and needs no assumption about how the bone was authored.
        if (current_dir.length() > kMinDirection && solved_dir.length() > kMinDirection) {
            after.basis = Basis(Quaternion(current_dir, solved_dir)) * before.basis;
        }
        // The leaf is r_bones[0] - the effector bone itself - not the last entry.
        // These modes are about the bone the effector drives, which is the one
        // GodotIK means by "the leaf".
        if (i == 0) {
            switch (p_effector->get_transform_mode()) {
                case FabrikEffector::PRESERVE_ROTATION: {
                    // Explicitly refuse any rotation change on the leaf. Same
                    // shape as GodotIK's PRESERVE_ROTATION.
                    after.basis = before.basis;
                } break;
                case FabrikEffector::FULL_TRANSFORM: {
                    after.basis = (p_skeleton->get_global_transform().affine_inverse() * p_effector->get_global_transform()).basis;
                } break;
                default:
                    break;
            }
        }
        new_global[bone] = after;
        touched[bone] = true;
    }
    // Report the chain root-first, matching the core's convention that
    // joints[0] is the root and joints[count-1] is the tip. r_bones is
    // leaf-first, so walking it backwards walks root-first; the solved array is
    // already root-first, so its index runs the other way (count - 1 - i, the
    // same solved_index the write loop uses). Appending solved[i] instead pairs
    // a root bone with the tip position, and the test caught exactly that: a
    // perfect solve reported as a tip sitting on the root.
    for (int32_t i = count - 1; i >= 0; --i) {
        last_bones.append(r_bones[i]);
        last_solved_positions.append(solved[count - 1 - i]);
    }
    return status;
}

void FabrikModifier3D::_write_chain(const PackedInt32Array &r_bones, FabrikEffector::TransformMode p_mode, Skeleton3D *p_skeleton) {
    const int32_t count = r_bones.size();
    // Root first: a bone's pose is its global transform relative to its parent's,
    // and `_write_pose` reads the parent's *new* transform, so the parent has to
    // be in place first. Measured: writing leaf-first leaves every bone off by
    // the parent's own delta.
    for (int32_t i = count - 1; i >= 0; --i) {
        const int32_t bone = r_bones[i];
        if (i == 0 && p_mode == FabrikEffector::STRAIGHTEN_CHAIN) {
            // No rotation of its own in the pose, so the chain's last segment
            // continues the parent bone's direction. Written here rather than at
            // solve time because it replaces the pose, and the pose is only
            // correct once the parent is final.
            const int32_t parent = p_skeleton->get_bone_parent(bone);
            if (parent >= 0) {
                const Transform3D parent_global = _current_global(parent, p_skeleton);
                Transform3D local = parent_global.affine_inverse() * new_global[bone];
                local.basis = Basis().scaled(local.basis.get_scale());
                p_skeleton->set_bone_pose(bone, local);
                new_global[bone] = parent_global * local;
                return;
            }
        }
        _write_pose(bone, new_global[bone], p_skeleton);
    }
}

int32_t FabrikModifier3D::solve_now() {
    if (solving) {
        return -1; // Already solving; the engine's update and this are one path.
    }
    Skeleton3D *skeleton = Object::cast_to<Skeleton3D>(get_skeleton());
    if (skeleton == nullptr) {
        last_error = "no Skeleton3D above the modifier";
        last_chain_count = 0;
        return -1;
    }
    solving = true;
    last_statuses.clear();
    last_bones.clear();
    last_solved_positions.clear();
    last_error = String();
    last_chain_count = 0;
    last_max_residual = 0.0f;
    _ensure_capacity(skeleton->get_bone_count());
    for (int32_t i = 0; i < skeleton->get_bone_count(); ++i) {
        touched[i] = false;
    }

    const TypedArray<FabrikEffector> effectors = _collect_effectors();
    std::vector<PackedInt32Array> pending_bones;
    std::vector<FabrikEffector::TransformMode> pending_modes;
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
        pending_bones.push_back(bones);
        pending_modes.push_back(effector->get_transform_mode());
        if (residual > last_max_residual) {
            last_max_residual = residual;
        }
    }

    // Every chain is solved before anything is written, so a bone whose parent
    // belongs to another chain reads that chain's NEW transform rather than a
    // stale one.
    for (int32_t i = 0; i < int32_t(pending_bones.size()); ++i) {
        _write_chain(pending_bones[i], pending_modes[i], skeleton);
    }

    // A manual solve has to reach the mesh now, because nothing else will run
    // this frame. A solve from inside the engine's own modifier callback must
    // NOT: forcing a transform update there re-enters the update in progress.
    if (!in_modifier_callback) {
        skeleton->force_update_all_bone_transforms();
    }
    solving = false;
    return last_chain_count;
}

void FabrikModifier3D::_process_modification() {
    if (solving) {
        return;
    }
    in_modifier_callback = true;
    solve_now();
    in_modifier_callback = false;
}

int32_t FabrikModifier3D::get_last_chain_count() const {
    return last_chain_count;
}

PackedInt32Array FabrikModifier3D::get_last_statuses() const {
    return last_statuses;
}

PackedInt32Array FabrikModifier3D::get_last_bones() const {
    return last_bones;
}

PackedVector3Array FabrikModifier3D::get_last_solved_positions() const {
    return last_solved_positions;
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
    ClassDB::bind_method(D_METHOD("get_last_bones"), &FabrikModifier3D::get_last_bones);
    ClassDB::bind_method(D_METHOD("get_last_solved_positions"), &FabrikModifier3D::get_last_solved_positions);
    ClassDB::bind_method(D_METHOD("get_last_error"), &FabrikModifier3D::get_last_error);
    ClassDB::bind_method(D_METHOD("get_last_max_residual"), &FabrikModifier3D::get_last_max_residual);

    ADD_PROPERTY(PropertyInfo(Variant::INT, "iteration_count", PROPERTY_HINT_RANGE, "1,64,1"), "set_iteration_count", "get_iteration_count");
}
