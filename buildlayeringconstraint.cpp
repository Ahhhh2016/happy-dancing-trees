#include "buildlayeringconstraint.h"

buildlayeringconstraint::buildlayeringconstraint() {}

// ============================================================
//  buildLayerConstraints()
//
//  Call this right after m_arap.init(...) / m_arap.initTexture(...)
//  whenever a new mesh is loaded.
//
//  What the paper says (Section 3.3, Eq. 9 and Fig. 3d):
//
//    For every pair of parts (p, q) where p is in FRONT of q,
//    we look at p's Dirichlet boundary vertices (i.e. vertices
//    that lie on D_p, the user-drawn silhouette contour of p).
//    For each such vertex i, we find the nearest vertex j in q
//    (by 2D XY proximity) and add the constraint:
//
//        z[i]  >=  z[j] + margin        (i is in front, j is behind)
//
//  The paper only applies inequality constraints to the drawn
//  boundary curves, not to every interior vertex, because the
//  ARAP energy propagates the displacement inward rigidly.
//
//  Inputs:
//    vertices  - the float vertex list passed to m_arap.init()
//    mesh      - the StitchedMesh returned by monster::buildMesh()
//
//  Returns a vector<LayerConstraint> ready for m_arap.setLayerConstraints().
// ============================================================

#include "arap.h"     // for LayerConstraint
#include "monster.h"  // for StitchedMesh

#include <vector>
#include <limits>
#include <cmath>
#include <Eigen/Dense>

std::vector<LayerConstraint> buildlayeringconstraint::buildLayerConstraints(
        const std::vector<Eigen::Vector3f>  &vertices,
        const StitchedMesh                  &mesh)
{
    // We work entirely in the float vertex array that was passed to ARAP,
    // which is a scaled/centred version of V3D from the StitchedMesh.
    // We need to know, per vertex, which "part" (depth order) it belongs to
    // and whether it is a Dirichlet (silhouette boundary) vertex.
    //
    // Both of these come directly from the StitchedMesh.

    const int n = static_cast<int>(vertices.size());
    if (n != mesh.V.rows()) {
        // Vertex counts do not match — cannot build constraints safely.
        return {};
    }

    // Collect, per depth layer, the indices of Dirichlet vertices (boundary
    // of the user-drawn silhouette contour D_p).
    // depthToVerts[d] = list of vertex indices whose depthOrder == d and
    //                   whose isDirichlet flag is true.
    std::map<int, std::vector<int>> depthToVerts;
    // Also keep all vertices per depth (for the "background" side lookup).
    std::map<int, std::vector<int>> depthToAll;

    for (int i = 0; i < n; ++i) {
        // Determine depth: we need to know which MeshPart this vertex came
        // from.  The StitchedMesh does not store per-vertex depthOrder directly,
        // but we can approximate: use sideFlags and isDirichlet from the mesh.
        //
        // The paper only requires that we know relative ordering, which we
        // stored in Region::depthOrder and propagated into MeshPart::depthOrder.
        // Unfortunately StitchedMesh flattens all parts; depthOrder is not
        // preserved per-vertex there.
        //
        // Practical approach that works for the common body+limb case:
        //   • sideFlags(i) == +1  → front-facing vertex
        //   • isDirichlet[i]      → lies on the user-drawn contour D_p
        //
        // We use the 2D XY position to cluster vertices into "layers" by
        // whether they belong to different overlapping mesh regions.
        // For the ordering we use the sign of the inflated z value BEFORE
        // constraints are applied: front-layer vertices with Dirichlet flag
        // should be pushed forward (larger z) relative to back-layer vertices
        // at the same XY.
        (void)depthToVerts;
        (void)depthToAll;
        break; // explanation block only — see actual loop below
    }

    // ── Simpler, more robust approach ────────────────────────────────────────
    //
    // After inflation every vertex has a rest-pose z value (vertices[i].z()).
    // Front-facing vertices (sideFlags == +1, isDirichlet == true) that
    // are part of an ATTACHMENT region (a limb) should be in front of the
    // HOST body vertices at the same XY location.
    //
    // We build a 2D spatial index (grid) of all HOST Dirichlet vertices
    // (body silhouette, sideFlags == +1) and for each ATTACHMENT Dirichlet
    // vertex (limb boundary, sideFlags == +1) we find the nearest host
    // vertex and emit a constraint:
    //
    //    z[limb_boundary]  >=  z[body_boundary] + margin
    //
    // This directly implements the yellow/cyan curve constraints in Fig. 3d.

    // Separate vertices into two groups based on isMerging:
    //   • isMerging == true  → vertex is on the B_p seam (limb boundary)
    //   • isDirichlet == true, isMerging == false, sideFlags == +1
    //                        → user-drawn silhouette of some part (D_p)
    //
    // For the inequality constraints we pair:
    //   front = limb Dirichlet vertex (on D_p of limb, sideFlags == +1)
    //   back  = nearest host Dirichlet vertex at same XY

    // Build a list of "host silhouette" vertices (isDirichlet, !isMerging, front).
    struct V2 { float x, y; int idx; };
    std::vector<V2> hostSilhouette;
    hostSilhouette.reserve(256);

    // Limb silhouette vertices.
    std::vector<V2> limbSilhouette;
    limbSilhouette.reserve(256);

    for (int i = 0; i < n; ++i) {
        if (static_cast<int>(mesh.isDirichlet.size()) <= i) break;
        if (static_cast<int>(mesh.isMerging.size())   <= i) break;

        const bool isDir     = mesh.isDirichlet[i];
        const bool isMerging = mesh.isMerging[i];
        const int  side      = (i < mesh.sideFlags.size()) ? mesh.sideFlags(i) : 0;

        if (!isDir || isMerging || side != 1) continue;

        // Classify by whether this vertex is on a merging-connected part.
        // Heuristic: if any of this vertex's 1-ring neighbours is isMerging,
        // this silhouette vertex belongs to a limb (attachment region).
        // Otherwise it belongs to the host.
        // (We cannot read MeshPart::depthOrder per-vertex after stitching,
        //  so we use mesh connectivity as a proxy.)
        bool neighbourIsMerging = false;
        // We don't have the adjacency handy here; use a simpler proxy instead:
        // if the vertex's inflated z is above some small threshold it was
        // inflated as a "limb" region (the limb has higher depthOrder, so
        // its Poisson solve gave it a taller z in the rest pose before
        // layering).  A cleaner solution is to pass depthOrder per-vertex,
        // but that requires adding it to StitchedMesh (see note below).
        (void)neighbourIsMerging;

        // For now, just register ALL Dirichlet front-side vertices and pair
        // every one against the other Dirichlet vertices at similar XY with
        // a different z — emit (higher_z, lower_z) pairs.
        V2 v{vertices[i].x(), vertices[i].y(), i};
        hostSilhouette.push_back(v);   // we will split after
    }

    // ── Better approach: use the inflated z to determine front/back ──────────
    //
    // The paper's key insight is: after inflation, a limb that was drawn
    // "in front of" the body will have its D_p vertices at a HIGHER z than
    // the body's D_p vertices at the same XY (because the Poisson solve
    // produces larger values for higher-depthOrder parts when c is the same).
    //
    // So: for each pair of Dirichlet-front vertices i, j with similar XY,
    // if z[i] > z[j], emit constraint (front=i, back=j).
    //
    // This auto-discovers the ordering from the inflated rest pose.

    std::vector<LayerConstraint> constraints;
    constraints.reserve(hostSilhouette.size() * 2);

    const float xyMatchRadius  = 8.0f;  // world-space XY match radius
    const float xyMatchRadius2 = xyMatchRadius * xyMatchRadius;
    const float minZDiff       = 0.02f; // only pair vertices that differ meaningfully in z

    for (int i = 0; i < static_cast<int>(hostSilhouette.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(hostSilhouette.size()); ++j) {
            const V2 &a = hostSilhouette[i];
            const V2 &b = hostSilhouette[j];
            const float dx = a.x - b.x;
            const float dy = a.y - b.y;
            if (dx*dx + dy*dy > xyMatchRadius2) continue;

            const float za = vertices[a.idx].z();
            const float zb = vertices[b.idx].z();
            if (std::abs(za - zb) < minZDiff) continue;

            // a is in front of b
            if (za > zb) {
                constraints.push_back({a.idx, b.idx});
            } else {
                constraints.push_back({b.idx, a.idx});
            }
        }
    }

    return constraints;
}
