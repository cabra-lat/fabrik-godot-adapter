#ifndef FABRIK_RIG_3D_H
#define FABRIK_RIG_3D_H

#include "fabrik_chain_3d.h"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/typed_array.hpp>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace godot {

// Orders and drives a set of FabrikChain3D objects as one unit.
//
// A full-body rig is not a set of independent chains: a hand target usually
// lives somewhere on an arm that has already been solved this frame, so "solve
// order" is part of the contract, not an implementation detail. This class owns
// an explicit, ordered list of chains plus optional "parent before child"
// dependencies, and solves them in a deterministic order derived from both.
//
// What it does NOT do, deliberately:
//   - no closed loops: a dependency cycle is reported, not relaxed;
//   - no constraint projection across chains: each chain is still solved alone;
//   - no scene-tree ownership: this is a RefCounted, not a Node.
class FabrikRig3D : public RefCounted {
    GDCLASS(FabrikRig3D, RefCounted)

    // Declaration order. Ties in the resolved order fall back to this order, so
    // a rig with no dependencies always solves exactly as declared.
    TypedArray<FabrikChain3D> chains;

    // Dependency edges keyed by instance id rather than by position: move_chain
    // and remove_chain then cannot leave a stale index behind.
    std::unordered_map<uint64_t, std::vector<uint64_t>> dependencies;
    std::unordered_map<uint64_t, Callable> target_providers;

    PackedInt32Array last_solve_order;
    TypedArray<int32_t> last_statuses;
    int32_t last_provider_failures = 0;
    String last_error;

public:
    // Membership. Duplicates and null chains are ignored so a rig cannot end up
    // solving the same chain twice per frame.
    void add_chain(const Ref<FabrikChain3D> &p_chain);
    bool remove_chain(const Ref<FabrikChain3D> &p_chain);
    void clear();
    TypedArray<FabrikChain3D> get_chains() const;
    int32_t get_chain_count() const;
    int32_t get_chain_index(const Ref<FabrikChain3D> &p_chain) const;
    // Explicit reordering, used as the tie-break when no dependency decides.
    bool move_chain(const Ref<FabrikChain3D> &p_chain, int32_t p_to_index);

    // "Solve p_parent before p_child." Both chains must already be in the rig
    // and must be different chains; anything else returns false.
    bool add_dependency(const Ref<FabrikChain3D> &p_child, const Ref<FabrikChain3D> &p_parent);
    bool remove_dependency(const Ref<FabrikChain3D> &p_child, const Ref<FabrikChain3D> &p_parent);

    // Called as provider(rig, index) right before that chain's solve, so a
    // target can be derived from a chain solved earlier in the same frame. Must
    // return a Vector3. An empty Callable clears the provider.
    void set_target_provider(const Ref<FabrikChain3D> &p_chain, const Callable &p_provider);
    void clear_target_provider(const Ref<FabrikChain3D> &p_chain);

    // False when the dependencies contain a cycle, or when a chain has been
    // freed out from under the rig. Nothing is solved in that case: a partial
    // solve would leave the caller with a pose nobody asked for.
    bool has_valid_order() const;
    bool solve_all();

    PackedInt32Array get_solve_order() const;
    TypedArray<int32_t> get_last_statuses() const;
    int32_t get_last_provider_failures() const;
    String get_last_error() const;

protected:
    static void _bind_methods();

private:
    // Declaration indices in the order they must be solved. Returns false and
    // leaves r_order untouched when the graph is cyclic.
    bool _compute_order(std::vector<int32_t> &r_order) const;
};

} // namespace godot

#endif
