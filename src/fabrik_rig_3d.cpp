#include "fabrik_rig_3d.h"


#include "fabrik_core.h"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/variant.hpp>

#include <algorithm>

using namespace godot;

namespace {

inline uint64_t chain_id(const Ref<FabrikChain3D> &p_chain) {
    return static_cast<uint64_t>(p_chain->get_instance_id());
}

} // namespace

void FabrikRig3D::add_chain(const Ref<FabrikChain3D> &p_chain) {
    if (p_chain.is_null()) {
        return;
    }
    // A chain may only appear once: solving it twice per frame would smooth it
    // twice and quietly make its easing frame-rate dependent.
    for (int64_t i = 0; i < chains.size(); ++i) {
        Ref<FabrikChain3D> existing = chains[i];
        if (existing.is_valid() && chain_id(existing) == chain_id(p_chain)) {
            return;
        }
    }
    chains.push_back(p_chain);
}

bool FabrikRig3D::remove_chain(const Ref<FabrikChain3D> &p_chain) {
    if (p_chain.is_null()) {
        return false;
    }
    const uint64_t id = chain_id(p_chain);
    for (int64_t i = 0; i < chains.size(); ++i) {
        Ref<FabrikChain3D> existing = chains[i];
        if (existing.is_valid() && chain_id(existing) == id) {
            chains.remove_at(i);
            // Edges are keyed by instance id, so a removed chain has to take its
            // own edges with it; otherwise the next solve would order against a
            // chain that is no longer in the rig.
            dependencies.erase(id);
            for (auto it = dependencies.begin(); it != dependencies.end();) {
                auto &parents = it->second;
                parents.erase(std::remove(parents.begin(), parents.end(), id), parents.end());
                if (parents.empty()) {
                    it = dependencies.erase(it);
                } else {
                    ++it;
                }
            }
            target_providers.erase(id);
            return true;
        }
    }
    return false;
}

void FabrikRig3D::clear() {
    chains.clear();
    dependencies.clear();
    target_providers.clear();
    last_solve_order = PackedInt32Array();
    last_statuses.clear();
    last_provider_failures = 0;
    last_error = String();
}

TypedArray<FabrikChain3D> FabrikRig3D::get_chains() const {
    return chains;
}

int32_t FabrikRig3D::get_chain_count() const {
    return static_cast<int32_t>(chains.size());
}

int32_t FabrikRig3D::get_chain_index(const Ref<FabrikChain3D> &p_chain) const {
    if (p_chain.is_null()) {
        return -1;
    }
    const uint64_t id = chain_id(p_chain);
    for (int64_t i = 0; i < chains.size(); ++i) {
        Ref<FabrikChain3D> existing = chains[i];
        if (existing.is_valid() && chain_id(existing) == id) {
            return static_cast<int32_t>(i);
        }
    }
    return -1;
}

bool FabrikRig3D::move_chain(const Ref<FabrikChain3D> &p_chain, int32_t p_to_index) {
    const int32_t from = get_chain_index(p_chain);
    const int32_t count = get_chain_count();
    if (from < 0 || count == 0) {
        return false;
    }
    const int32_t to = CLAMP(p_to_index, 0, count - 1);
    if (to == from) {
        return true;
    }
    // TypedArray is a view onto shared array storage (same as Array), so take a
    // real copy before mutating: a rotate could otherwise operate on the array
    // it is reading from.
    TypedArray<FabrikChain3D> reordered = Array(chains.duplicate());
    const Ref<FabrikChain3D> moved = reordered[from];
    reordered.remove_at(from);
    reordered.insert(to, moved);
    chains = reordered;
    return true;
}

bool FabrikRig3D::add_dependency(const Ref<FabrikChain3D> &p_child, const Ref<FabrikChain3D> &p_parent) {
    if (p_child.is_null() || p_parent.is_null()) {
        return false;
    }
    if (chain_id(p_child) == chain_id(p_parent)) {
        return false;
    }
    if (get_chain_index(p_child) < 0 || get_chain_index(p_parent) < 0) {
        return false;
    }
    std::vector<uint64_t> &parents = dependencies[chain_id(p_child)];
    const uint64_t parent_id = chain_id(p_parent);
    if (std::find(parents.begin(), parents.end(), parent_id) != parents.end()) {
        return true;
    }
    parents.push_back(parent_id);
    return true;
}

bool FabrikRig3D::remove_dependency(const Ref<FabrikChain3D> &p_child, const Ref<FabrikChain3D> &p_parent) {
    if (p_child.is_null() || p_parent.is_null()) {
        return false;
    }
    auto it = dependencies.find(chain_id(p_child));
    if (it == dependencies.end()) {
        return false;
    }
    std::vector<uint64_t> &parents = it->second;
    const auto found = std::find(parents.begin(), parents.end(), chain_id(p_parent));
    if (found == parents.end()) {
        return false;
    }
    parents.erase(found);
    if (parents.empty()) {
        dependencies.erase(it);
    }
    return true;
}

void FabrikRig3D::set_target_provider(const Ref<FabrikChain3D> &p_chain, const Callable &p_provider) {
    if (p_chain.is_null() || get_chain_index(p_chain) < 0) {
        return;
    }
    if (p_provider.is_null()) {
        target_providers.erase(chain_id(p_chain));
        return;
    }
    target_providers[chain_id(p_chain)] = p_provider;
}

void FabrikRig3D::clear_target_provider(const Ref<FabrikChain3D> &p_chain) {
    if (p_chain.is_null()) {
        return;
    }
    target_providers.erase(chain_id(p_chain));
}

bool FabrikRig3D::_compute_order(std::vector<int32_t> &r_order) const {
    const int32_t count = get_chain_count();
    r_order.clear();
    if (count == 0) {
        return true;
    }

    std::unordered_map<uint64_t, int32_t> index_of;
    for (int32_t i = 0; i < count; ++i) {
        Ref<FabrikChain3D> chain = chains[i];
        if (chain.is_null()) {
            // Freed underneath us: not a cycle, but not orderable either.
            return false;
        }
        index_of.emplace(chain_id(chain), i);
    }

    // The ordering is Kahn's algorithm and it lives in the core, so it is
    // covered by `fpm test` and the sanitizer rather than only by a GDScript
    // test. What stays here is the Godot-shaped part: turning chain object ids
    // into integer edges, then handing back an order.
    //
    // Three rules are applied while flattening, and all three belong here rather
    // than in the core because they are about object identity, not graph theory:
    // a chain cannot depend on itself, a dependency on a chain that has left the
    // rig is not a constraint on this one, and a repeated edge must not be
    // counted twice or the child would wait for a parent released only once.
    std::vector<int32_t> flat_edges;
    flat_edges.reserve(static_cast<size_t>(count) * 2U);
    for (int32_t i = 0; i < count; ++i) {
        const Ref<FabrikChain3D> chain = chains[i];
        const auto edges = dependencies.find(chain_id(chain));
        if (edges == dependencies.end()) {
            continue;
        }
        std::vector<uint64_t> counted;
        for (uint64_t parent_id : edges->second) {
            const auto parent = index_of.find(parent_id);
            if (parent == index_of.end() || parent->second == i) {
                continue;
            }
            if (std::find(counted.begin(), counted.end(), parent_id) != counted.end()) {
                continue;
            }
            counted.push_back(parent_id);
            flat_edges.push_back(i);
            flat_edges.push_back(parent->second);
        }
    }
    const int32_t pairs = static_cast<int32_t>(flat_edges.size() / 2U);

    r_order.assign(static_cast<size_t>(count), -1);
    if (fabrik_order_dependencies_f32(count, flat_edges.data(), pairs, r_order.data()) != FABRIK_OK) {
        // A cycle, which the core refuses wholesale rather than returning a
        // truncated order that would solve half the rig.
        r_order.clear();
        return false;
    }
    return true;
}

bool FabrikRig3D::has_valid_order() const {
    std::vector<int32_t> order;
    return _compute_order(order);
}

bool FabrikRig3D::solve_all() {
    last_error = String();
    last_provider_failures = 0;
    last_statuses.clear();

    std::vector<int32_t> order;
    if (!_compute_order(order)) {
        // Nothing is solved on purpose: a half-solved rig looks posed but is not,
        // and the caller would have to guess which half is trustworthy.
        last_solve_order = PackedInt32Array();
        last_error = "dependency cycle or freed chain: nothing was solved";
        return false;
    }

    PackedInt32Array order_array;
    order_array.resize(static_cast<int32_t>(order.size()));
    for (size_t i = 0; i < order.size(); ++i) {
        order_array[static_cast<int32_t>(i)] = order[i];
    }

    for (int32_t index : order) {
        Ref<FabrikChain3D> chain = chains[index];
        if (chain.is_null()) {
            last_solve_order = PackedInt32Array();
            last_error = "a chain was freed during the solve";
            return false;
        }
        const auto provider = target_providers.find(chain_id(chain));
        if (provider != target_providers.end() && provider->second.is_valid()) {
            // Ref for `this` only for the duration of the call: the callable
            // receives the rig so it can read chains solved earlier this frame.
            Ref<FabrikRig3D> self(this);
            Array arguments;
            arguments.push_back(self);
            arguments.push_back(index);
            const Variant result = provider->second.callv(arguments);
            if (result.get_type() == Variant::VECTOR3) {
                const Vector3 target = result;
                chain->set_target(target);
            } else {
                // Counted, not fatal: the chain still solves, using the target it
                // already had. Silently swallowing this would look like the
                // provider simply had nothing to say this frame.
                last_provider_failures += 1;
            }
        }
        last_statuses.push_back(chain->solve());
    }

    last_solve_order = order_array;
    return true;
}

PackedInt32Array FabrikRig3D::get_solve_order() const {
    return last_solve_order;
}

TypedArray<int32_t> FabrikRig3D::get_last_statuses() const {
    return last_statuses;
}

int32_t FabrikRig3D::get_last_provider_failures() const {
    return last_provider_failures;
}

String FabrikRig3D::get_last_error() const {
    return last_error;
}

void FabrikRig3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("add_chain", "chain"), &FabrikRig3D::add_chain);
    ClassDB::bind_method(D_METHOD("remove_chain", "chain"), &FabrikRig3D::remove_chain);
    ClassDB::bind_method(D_METHOD("clear"), &FabrikRig3D::clear);
    ClassDB::bind_method(D_METHOD("get_chains"), &FabrikRig3D::get_chains);
    ClassDB::bind_method(D_METHOD("get_chain_count"), &FabrikRig3D::get_chain_count);
    ClassDB::bind_method(D_METHOD("get_chain_index", "chain"), &FabrikRig3D::get_chain_index);
    ClassDB::bind_method(D_METHOD("move_chain", "chain", "to_index"), &FabrikRig3D::move_chain);
    ClassDB::bind_method(D_METHOD("add_dependency", "child", "parent"), &FabrikRig3D::add_dependency);
    ClassDB::bind_method(D_METHOD("remove_dependency", "child", "parent"), &FabrikRig3D::remove_dependency);
    ClassDB::bind_method(D_METHOD("set_target_provider", "chain", "provider"), &FabrikRig3D::set_target_provider, DEFVAL(Callable()));
    ClassDB::bind_method(D_METHOD("clear_target_provider", "chain"), &FabrikRig3D::clear_target_provider);
    ClassDB::bind_method(D_METHOD("has_valid_order"), &FabrikRig3D::has_valid_order);
    ClassDB::bind_method(D_METHOD("solve_all"), &FabrikRig3D::solve_all);
    ClassDB::bind_method(D_METHOD("get_solve_order"), &FabrikRig3D::get_solve_order);
    ClassDB::bind_method(D_METHOD("get_last_statuses"), &FabrikRig3D::get_last_statuses);
    ClassDB::bind_method(D_METHOD("get_last_provider_failures"), &FabrikRig3D::get_last_provider_failures);
    ClassDB::bind_method(D_METHOD("get_last_error"), &FabrikRig3D::get_last_error);
}
