#ifndef FABRIK_CHAIN_3D_H
#define FABRIK_CHAIN_3D_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace godot {

class FabrikChain3D : public RefCounted {
    GDCLASS(FabrikChain3D, RefCounted)

    PackedVector3Array joints;
    PackedFloat32Array segment_lengths;
    Vector3 target;
    bool root_anchored = true;
    float tolerance = 0.00001f;
    int32_t max_iterations = 64;
    int32_t last_status = 0;
    float last_residual = 0.0f;

public:
    void set_joints(const PackedVector3Array &p_joints);
    PackedVector3Array get_joints() const;
    void set_segment_lengths(const PackedFloat32Array &p_lengths);
    PackedFloat32Array get_segment_lengths() const;
    void set_target(const Vector3 &p_target);
    Vector3 get_target() const;
    void set_root_anchored(bool p_anchored);
    bool get_root_anchored() const;
    void set_tolerance(float p_tolerance);
    float get_tolerance() const;
    void set_max_iterations(int32_t p_iterations);
    int32_t get_max_iterations() const;

    int32_t solve();
    int32_t get_last_status() const;
    float get_last_residual() const;
    String get_last_status_name() const;

protected:
    static void _bind_methods();
};

} // namespace godot

#endif
