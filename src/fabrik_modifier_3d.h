#ifndef FABRIK_MODIFIER_3D_H
#define FABRIK_MODIFIER_3D_H

#include "fabrik_chain_3d.h"
#include "fabrik_effector.h"

#include <godot_cpp/classes/skeleton_modifier3d.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/typed_array.hpp>

#include <cstdint>
#include <vector>

namespace godot {

// Drives FabrikEffector children over a Skeleton3D, the way GodotIK's GodotIK
// does. Being a SkeletonModifier3D is the whole point: the engine calls
// _process_modification() every skeleton update, so a scene needs no per-frame
// script, and a skeleton can stack this under an AnimationPlayer.
//
// What this does, and what it still does not do, is tracked honestly in
// docs/GODOTIK_COMPATIBILITY.md. In short: chains are built by walking
// get_bone_parent() upwards from each effector bone (so a GodotIK scene's
// chain topology is the same), influence is a blend from the bone's current
// position towards the effector, and results are written back as bone poses.
// Closed loops, leaf joints, ancestor propagation, custom constraints and
// transform-mode parity beyond the four enum values are NOT implemented.
//
// Everything happens in the space Skeleton3D::get_bone_global_pose() reports,
// which is measured to be the skeleton's own space, not world space: the
// effector target is converted in with Skeleton3D::to_local() and the results
// are converted out with the parent bone's global transform. Results are
// written as pose = parent_global.inverse() * new_global, which is the
// relationship Skeleton3D itself uses.
class FabrikModifier3D : public SkeletonModifier3D {
    GDCLASS(FabrikModifier3D, SkeletonModifier3D)

    // GodotIK defaults to 8 sweeps per frame. Kept identical on purpose: a
    // hotswap should not change the pose just because the default changed.
    int32_t iteration_count = 8;
    PackedInt32Array last_statuses;
    int32_t last_chain_count = 0;
    // Largest residual over the chains solved last frame, in the core's own
    // units. A non-zero value is the honest answer to "did it get there?": a
    // target outside the reach sphere leaves the tip short on purpose.
    float last_max_residual = 0.0f;
    String last_error;

    // Per-bone working set, indexed by bone index. `touched` marks bones this
    // solve moved, so a parent that another chain already moved is picked up
    // instead of its stale snapshot.
    std::vector<Transform3D> new_global;
    std::vector<Transform3D> snapshot_global;
    std::vector<bool> touched;

public:
    void set_iteration_count(int32_t p_count);
    int32_t get_iteration_count() const;

    // The FabrikEffector children, in tree order. A GodotIK scene holds
    // GodotIKEffector children instead, which this cannot see.
    TypedArray<FabrikEffector> get_effectors() const;

    // Solve now. Returns the number of chains solved, or -1 when there is no
    // skeleton. The engine calls this from _process_modification(); it is
    // public so a test, or a manual pipeline, can drive it directly.
    int32_t solve_now();

    int32_t get_last_chain_count() const;
    // The core's status per chain solved, in solve order. Anything other than
    // 0 (OK) means the chain did not reach its goal - see the core's header for
    // the codes.
    PackedInt32Array get_last_statuses() const;
    float get_last_max_residual() const;
    // Human-readable reason the last solve skipped a chain, or "" when all
    // chains were solved.
    String get_last_error() const;

    void _process_modification() override;

protected:
    static void _bind_methods();

private:
    TypedArray<FabrikEffector> _collect_effectors() const;
    // Bones from the effector bone outwards, or an empty array when the chain
    // cannot be built (unknown bone, a bone with no parent, or a chain too
    // short to bend).
    PackedInt32Array _build_bone_chain(const FabrikEffector *p_effector, Skeleton3D *p_skeleton) const;
    // Fills `new_global` for one chain. Returns the core's status code and
    // reports the chain's residual through `r_residual`. Writes nothing to the
    // skeleton: see _write_chain.
    int32_t _solve_chain(const FabrikEffector *p_effector, Skeleton3D *p_skeleton, const PackedInt32Array &r_bones, float &r_residual);
    // Applies `new_global` to the skeleton for one chain, root first. A bone's
    // pose is its global transform relative to its parent's, so a parent that is
    // about to move must be written first - writing leaf-first silently lands
    // every bone relative to a transform that is then changed under it.
    void _write_chain(const PackedInt32Array &r_bones, FabrikEffector::TransformMode p_mode, Skeleton3D *p_skeleton);
    void _ensure_capacity(int32_t p_bone_count);
    Transform3D _current_global(int32_t p_bone, Skeleton3D *p_skeleton) const;
    void _write_pose(int32_t p_bone, const Transform3D &p_global, Skeleton3D *p_skeleton);
};

} // namespace godot

#endif
