#ifndef FABRIK_CHAIN_3D_H
#define FABRIK_CHAIN_3D_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace godot {

class FabrikChain3D : public RefCounted {
    GDCLASS(FabrikChain3D, RefCounted)

    PackedVector3Array joints;
    PackedFloat32Array segment_lengths;
    Vector3 target;
    // Optional pole hint. FABRIK solves positions and leaves the bend plane
    // underdetermined; a non-zero pole target rotates the solved intermediate
    // joints around the root-to-tip axis so the elbow/knee faces a chosen side.
    Vector3 pole_target;
    bool root_anchored = true;
    float tolerance = 0.00001f;
    int32_t max_iterations = 64;
    // 0 = use the raw solve, 1 = never move. Applied per solve so a moving
    // target does not make the chain snap between poses every frame.
    float smoothing = 0.0f;
    // Optional per-joint angle limits, in DEGREES, given as the INTERIOR angle
    // between the incoming and the outgoing segment at a joint: 180 is straight,
    // 0 is folded back on itself, so an elbow that may not hyperextend reads as
    // (0, 150). The array is indexed by joint; joints without an entry, and
    // joints whose entry is exactly (0, 180), are unlimited.
    PackedVector2Array joint_limits;
    // A limit is enforced by rotating the sub-chain below the joint, which can
    // introduce a violation at the next joint. A few passes settle the common
    // cases; anything still outside its range is reported rather than hidden.
    int32_t limit_iterations = 4;
    int32_t last_limit_projections = 0;
    int32_t last_limit_violations = 0;
    int32_t last_status = 0;
    float last_residual = 0.0f;
    // Godot has no PackedQuaternionArray, so rotations are a typed Array.
    // mutable because _update_rotations() is a const query: the cached result is
    // derived state, and Array::resize/set are non-const in godot-cpp.
    mutable TypedArray<Quaternion> last_rotations;

public:
    void set_joints(const PackedVector3Array &p_joints);
    PackedVector3Array get_joints() const;
    void set_segment_lengths(const PackedFloat32Array &p_lengths);
    PackedFloat32Array get_segment_lengths() const;
    void set_target(const Vector3 &p_target);
    Vector3 get_target() const;
    void set_pole_target(const Vector3 &p_pole_target);
    Vector3 get_pole_target() const;
    void set_root_anchored(bool p_anchored);
    bool get_root_anchored() const;
    void set_tolerance(float p_tolerance);
    float get_tolerance() const;
    void set_max_iterations(int32_t p_iterations);
    int32_t get_max_iterations() const;
    void set_smoothing(float p_smoothing);
    float get_smoothing() const;

    // Angle limits, enforced as a post-solve constraint. A limit makes the
    // target genuinely unreachable, so the tip ends up short of it: that is the
    // constraint doing its job, and the residual reports the shortfall.
    void set_joint_limits(const PackedVector2Array &p_limits);
    PackedVector2Array get_joint_limits() const;
    void clear_joint_limits();
    void set_limit_iterations(int32_t p_iterations);
    int32_t get_limit_iterations() const;
    // Measured interior angle per joint in degrees; the two ends report 0.
    PackedFloat32Array get_joint_angles() const;
    int32_t get_limit_projection_count() const;
    // Joints still outside their range after the last solve. 0 is the healthy
    // case; anything above it means the limits and the pose could not both hold.
    int32_t get_limit_violation_count() const;

    int32_t solve();
    int32_t get_last_status() const;
    float get_last_residual() const;
    String get_last_status_name() const;

    // Per-joint orientations derived from the solved positions. The core is a
    // position solver, so this is the adapter's job: each frame is parallel
    // transported along the chain so bones do not flip when the chain bends.
    // Convention: the bone points along local +Y, matching Skeleton3D bones.
    TypedArray<Quaternion> get_joint_rotations() const;
    TypedArray<Quaternion> get_last_joint_rotations() const;

    // Applies the current rotations to a Skeleton3D. Returns the number of
    // bones posed, or a negative value when the inputs do not line up.
    int32_t pose_skeleton(Skeleton3D *p_skeleton, const PackedStringArray &p_bone_names);

protected:
    static void _bind_methods();

private:
    void _update_rotations() const;
    // Interior angle at a joint, in degrees; 0 at the two ends.
    float _interior_angle_degrees(int32_t p_index) const;
    Vector3 _perpendicular_to(const Vector3 &p_direction) const;
    void _apply_pole_constraint();
    void _apply_joint_limits();
    void _recompute_residual();
};

} // namespace godot

#endif
