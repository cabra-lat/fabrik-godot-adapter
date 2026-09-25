#ifndef FABRIK_EFFECTOR_H
#define FABRIK_EFFECTOR_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/string.hpp>

namespace godot {

class Skeleton3D;

// A scene-tree effector for FabrikModifier3D: the node whose transform says
// where a chain should reach, and which bone that chain hangs off.
//
// Deliberately shaped like GodotIK's GodotIKEffector (Node3D; bone_name,
// chain_length, active, influence, transform_mode) so a scene built for one
// reads the same for the other. What is NOT compatible yet, and is documented
// in docs/GODOTIK_COMPATIBILITY.md:
//
//   - a GodotIK scene's effector children are instances of GodotIKEffector, and
//     a GDExtension cannot pass those off as FabrikEffector. Swapping means
//     retyping the effector nodes, not renaming them;
//   - GodotIKConstraint children are ignored here (the pole target is a
//     different mechanism).
//
// A FabrikEffector only means anything as a child of a FabrikModifier3D; the
// target it publishes is its own global_position, converted into the
// skeleton's space by the modifier.
class FabrikEffector : public Node3D {
    GDCLASS(FabrikEffector, Node3D)

public:
    // What to do with the leaf bone's rotation once the chain has been solved.
    // Mirrors GodotIKEffector::TransformMode so the two read alike; the
    // meanings are implemented identically, see FabrikModifier3D.
    enum TransformMode {
        POSITION_ONLY = 0, // Keep the rotation the solve produced.
        PRESERVE_ROTATION = 1, // Keep the leaf's pre-solve rotation too.
        STRAIGHTEN_CHAIN = 2, // Leaf pose gets no rotation of its own.
        FULL_TRANSFORM = 3, // Take the leaf's orientation from this node.
    };

private:
    String bone_name;
    int32_t bone_idx = -1;
    int32_t chain_length = 2;
    bool active = true;
    float influence = 1.0f;
    TransformMode transform_mode = POSITION_ONLY;
    // Optional node whose position is used as the chain's pole hint, i.e. which
    // side the elbow or knee should bend towards. Empty means no pole target.
    NodePath pole_target_path;

public:
    // The bone this effector drives. Setting the name resolves it immediately
    // against the controlling skeleton's bone list and updates bone_idx; a name
    // that matches no bone leaves bone_idx at -1 and the effector inert.
    void set_bone_name(const String &p_name);
    String get_bone_name() const;
    void set_bone_idx(int32_t p_bone_idx);
    int32_t get_bone_idx() const;
    // Bones in the chain, counting outwards from the effector bone. GodotIK
    // defaults to 2; a chain of 1 bone cannot be solved (it has no freedom to
    // bend), so the minimum useful value here is 2.
    void set_chain_length(int32_t p_chain_length);
    int32_t get_chain_length() const;
    void set_active(bool p_active);
    bool is_active() const;
    // 0 = leave the bone where it is, 1 = go all the way to the target. The
    // blend is measured against the bone's current position each frame, so
    // animating this ramps the solve in and out.
    void set_influence(float p_influence);
    float get_influence() const;
    void set_transform_mode(TransformMode p_transform_mode);
    TransformMode get_transform_mode() const;
    void set_pole_target_path(const NodePath &p_path);
    NodePath get_pole_target_path() const;
    // The resolved pole node, or nullptr when unset or dangling.
    Node3D *get_pole_target() const;

    // Re-resolves bone_idx from bone_name. The modifier calls this every solve,
    // so a bone added at runtime does not need a signal to be picked up.
    void refresh_bone_idx(Skeleton3D *p_skeleton);

protected:
    static void _bind_methods();
};

} // namespace godot

VARIANT_ENUM_CAST(FabrikEffector::TransformMode);

#endif
