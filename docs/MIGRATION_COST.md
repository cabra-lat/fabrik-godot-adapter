# The cost of replacing the live IK, and why the answer is no

This document exists to **close** a question, not to plan a migration. It is
written as an argument that the work is not worth doing, with the numbers that
make the argument. If you are looking for a plan to swap FABRIK into a project,
this is the wrong document.

The short version: replacing a working IK with this one costs a scene rewrite, a
constraint rewrite and a parity campaign, and buys nothing that has been
observed. That last clause is the important one — not "buys little", but "buys
nothing measurable yet".

## What was measured before this conclusion

The comparison that was never run is the one that would decide it. Here is what
exists instead:

- Both solvers have been characterised separately, from source and papers.
  `docs/GODOTIK_COMPATIBILITY.md` records the differences, with the GodotIK
  source paths that establish each one.
- The adapter's own behaviour is now negative-controlled: the engine hook, the
  space conversion and the pose write order were each disabled on a throwaway
  branch, and the test suite was required to go red. All three did.
- The live GodotIK in this project's game repo was measured functionally (PASS)
  and on frame time (P95 19.22 ms against a 16.7 ms budget). It passes and it is
  over budget.

**No one has run both solvers on the same rig and compared anything.** Every
replacement claim, favourable or unfavourable, is therefore extrapolation.

## Why the feature list does not justify a swap

A feature comparison favours FABRIK on paper:

| | GodotIK | this adapter |
| --- | --- | --- |
| per-joint angle limits | none built in | yes |
| pole target | none built in | yes |
| iteration count default | 8 | 8 (matched deliberately) |
| influence semantics | `lerp(current, effector, influence)` | identical, by construction |

Both of the "yes" rows are **post-solve projections**, not in-solver constraint
propagation. That distinction is not academic:

- A projection rotates the sub-chain below a joint into its limit and leaves the
  root and all segment lengths exact. The tip then falls short of the target, and
  the residual says so.
- The 2015 constraint paper (Aristidou, Chrysanthou & Lasenby) clamps the
  *target* into the allowable bounds at each iteration, so the solver converges
  on a target the chain can actually satisfy.

These produce different poses for the same inputs. Neither is wrong; they are
different contracts, and choosing between them is a project decision, not a
migration.

Meanwhile the rows that actually block a swap are all "no" here:

- **No custom constraints.** GodotIK exposes a per-bone virtual
  `apply(parent, bone, child, direction)`. Anything using it loses it.
- **No ancestor propagation or rest-pose handling.** GodotIK keeps
  `initial_transforms` and propagates from a chain's ancestor on every iteration.
  Here, a bone moved by two chains is written twice, the second write winning, so
  overlapping chains diverge from GodotIK even though each is correct alone.
- **No dual-run parity harness**, so item 8 above — how much the two solvers'
  poses differ on one skeleton — is unmeasured.

So: the features FABRIK has are the ones a project may not need, and the
features it lacks are the ones a real rig needs.

## The unavoidable costs

Even if a project wanted the two features it has, these costs are on the path
regardless:

1. **Effector nodes must be retyped.** A GDExtension cannot make its class pass
   an `is_class("GodotIKEffector")` test, so every effector in every scene and
   every prefab has to be re-authored. This is scene work, not code work, and it
   is the single largest item.
2. **Custom constraint nodes must be rewritten** to `FabrikConstraint`, which
   does not exist yet.
3. **A parity campaign must be run**: both solvers on one skeleton, per-bone
   deltas, at chain lengths 2, 3 and 4, plus frame cost. Until that exists,
   "hotswappable" is a design claim, not a measured one.
4. **Ancestor propagation and rest-pose handling must be implemented** before
   overlapping chains are safe at all.

Steps 2 and 4 are unimplemented feature work, not integration work. A migration
estimate that skips them is not an estimate.

## The one argument that would change the answer

If the live solver's frame time is the problem, then FABRIK's constant factors
matter, and the thing to measure is per-solve cost for a 3-bone chain — not
features, and not this document. That measurement is cheap and nobody has taken
it. If it showed a decisive win, the parity harness in step 3 would become worth
building.

Absent that number, the honest position is the one recorded above: **the
replacement is not justified, and this document is the reason.** The prototype is
worth keeping as a documented, tested reference implementation with two
constraint features the incumbent lacks. That is a different and smaller claim
than "a better production IK", and it is the one the evidence supports.
