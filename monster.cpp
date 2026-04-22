#include "monster.h"
#include <iostream>
#include <limits>
#include <igl/writeOBJ.h>
#include <igl/remove_unreferenced.h>

using namespace Eigen;
using namespace std;

monster::monster() {}

StitchedMesh monster::buildMesh(const std::vector<Region>& regions,
                                const std::vector<std::vector<int>>& connectedRegions) {
    // Classify regions by original index
    std::vector<int> hostIndices, attachmentIndices;
    for (int i = 0; i < (int)regions.size(); i++) {
        bool hasMerging = false;
        for (const Stroke& s : regions[i].boundaries)
            if (s.isMergingBoundary) { hasMerging = true; break; }
        if (hasMerging) attachmentIndices.push_back(i);
        else hostIndices.push_back(i);
    }
    std::cout << "Host regions: " << hostIndices.size() << std::endl;
    std::cout << "Attachment regions: " << attachmentIndices.size() << std::endl;

    // Helper: find which component contains a region index
    auto getComponent = [&](int regionIdx) -> std::vector<int> {
        for (const auto& comp : connectedRegions)
            for (int idx : comp)
                if (idx == regionIdx) return comp;
        return {};
    };

    // Helper: find host index for an attachment
    auto getHostForAttachment = [&](int ai) -> int {
        auto comp = getComponent(ai);
        for (int idx : comp)
            for (int hi : hostIndices)
                if (idx == hi) return hi;
        return -1;
    };

    // Triangulate host regions using combined triangulation
    std::map<int, std::vector<Eigen::Vector2f>> exactBpByAttachment;
    std::map<int, Eigen::MatrixXd> hostV2ByIndex;

    for (int hi : hostIndices) {
        const Region& host = regions[hi];

        // collect attachments for this host
        std::vector<const Region*> hostAttachments;
        std::vector<int> hostAttachmentIndices;
        for (int ai : attachmentIndices) {
            if (getHostForAttachment(ai) == hi) {
                hostAttachments.push_back(&regions[ai]);
                hostAttachmentIndices.push_back(ai);
            }
        }

        Eigen::MatrixXd V2;
        Eigen::MatrixXi F2;
        std::vector<bool> isDirichlet, isMerging;
        std::vector<std::pair<int,int>> armpitPairs;
        triangulateRegions(host, hostAttachments, V2, F2, isDirichlet, isMerging, armpitPairs);
        hostV2ByIndex[hi] = V2;

        // Extract exact Bp verts per attachment from host mesh
        for (int ai : hostAttachmentIndices) {
            auto rawBp = getMergingBoundaryPoints(regions[ai]);
            if (rawBp.size() < 2) continue;
            Eigen::Vector2d bp0 = rawBp.front().cast<double>();
            Eigen::Vector2d bp1 = rawBp.back().cast<double>();
            Eigen::Vector2d seg = bp1 - bp0;
            std::vector<Eigen::Vector2f> exactVerts;
            for (int i = 0; i < (int)V2.rows(); i++) {
                if (!isMerging[i]) continue;
                Eigen::Vector2d p = V2.row(i);
                // skip armpit endpoints
                if ((p - bp0).norm() < 0.5 || (p - bp1).norm() < 0.5) continue;
                // check on this attachment's Bp segment
                double t = (p - bp0).dot(seg) / seg.dot(seg);
                t = std::max(0.0, std::min(1.0, t));
                double dist = (p - (bp0 + t * seg)).norm();
                if (dist < 0.5)
                    exactVerts.push_back(V2.row(i).cast<float>());
            }
            if (exactVerts.empty()) {
                // no Steiner points — use midpoint
                exactVerts.push_back((rawBp.front() + rawBp.back()) / 2.0f);
                std::cout << "No interior Bp verts for attachment " << ai
                          << ", using midpoint" << std::endl;
            }
            exactBpByAttachment[ai] = exactVerts;
            std::cout << "Exact Bp verts for attachment " << ai << ": "
                      << exactVerts.size() << std::endl;
        }

        MeshPart part = createFrontBack(V2, F2, isDirichlet, host.depthOrder, isMerging);
        for (auto& ap : armpitPairs)
            part.armpitPairs.push_back(ap);
        m_meshParts.push_back(part);
    }

    // Triangulate attachment regions using exact Bp verts from host
    for (int ai : attachmentIndices) {
        const Region& region = regions[ai];
        auto rawBp = getMergingBoundaryPoints(region);

        // use exact Bp verts from host if available
        auto bpPoints = (!exactBpByAttachment[ai].empty())
                            ? exactBpByAttachment[ai]
                            : rawBp;

        std::cout << "Attachment " << ai << " using " << bpPoints.size()
                  << " Bp points" << std::endl;

        Eigen::MatrixXd V, V2;
        Eigen::MatrixXi F2;
        int n;
        triangulateRegion(region, V, n, V2, F2, bpPoints);
        auto isDirichlet = buildIsDirichlet(V2, V, n);

        // build isMerging using nearest neighbor to bpPoints
        std::vector<bool> isMerging(V2.rows(), false);
        for (int i = 0; i < (int)V2.rows(); i++) {
            Eigen::Vector2d p = V2.row(i);
            for (auto& bp : bpPoints) {
                double dx = p.x() - bp.x();
                double dy = p.y() - bp.y();
                if (dx*dx + dy*dy < 0.25) { // 0.5px tolerance
                    isMerging[i] = true;
                    break;
                }
            }
            // also check along full Bp segment
            if (!isMerging[i] && rawBp.size() >= 2) {
                Eigen::Vector2d bp0 = rawBp.front().cast<double>();
                Eigen::Vector2d bp1 = rawBp.back().cast<double>();
                Eigen::Vector2d seg = bp1 - bp0;
                double t = (p - bp0).dot(seg) / seg.dot(seg);
                t = std::max(0.0, std::min(1.0, t));
                double dist = (p - (bp0 + t * seg)).norm();
                if (dist < 0.5) isMerging[i] = true;
            }
        }

        MeshPart part = createFrontBack(V2, F2, isDirichlet, region.depthOrder, isMerging);

        // store armpit pairs for attachment endpoints
        for (int i = 0; i < (int)V2.rows(); i++) {
            if (!isMerging[i]) continue;
            Eigen::Vector2d p = V2.row(i);
            Eigen::Vector2d bp0 = rawBp.front().cast<double>();
            Eigen::Vector2d bp1 = rawBp.back().cast<double>();
            if ((p - bp0).norm() < 0.5 || (p - bp1).norm() < 0.5)
                part.armpitPairs.push_back({i, i + (int)V2.rows()});
        }
        m_meshParts.push_back(part);
    }

    StitchedMesh result = stitchParts();
    Eigen::MatrixXd V3D(result.V.rows(), 3);
    V3D << result.V, Eigen::VectorXd::Zero(result.V.rows());
    igl::writeOBJ("mesh12.obj", V3D, result.F);
    return result;
}

std::vector<bool> monster::buildIsMerging(const Eigen::MatrixXd& V,
                                 const std::vector<Eigen::Vector2f>& bpPoints,
                                 double eps) {
    std::vector<bool> isMerging(V.rows(), false);
    if (bpPoints.size() < 2) return isMerging;
    for (int i = 0; i < V.rows(); i++) {
        Eigen::Vector2d p = V.row(i);
        for (int j = 0; j + 1 < (int)bpPoints.size(); j++) {
            Eigen::Vector2d a = bpPoints[j].cast<double>();
            Eigen::Vector2d b = bpPoints[j+1].cast<double>();
            // distance from p to segment ab
            Eigen::Vector2d ab = b - a, ap = p - a;
            double t = ap.dot(ab) / ab.dot(ab);
            t = std::max(0.0, std::min(1.0, t));
            double dist = (p - (a + t * ab)).norm();
            if (dist < eps) { isMerging[i] = true; break; }
        }
    }
    return isMerging;
}

std::vector<Eigen::Vector2f> monster::getMergingBoundaryPoints(const Region& region) {
    for (const Stroke& stroke : region.boundaries) {
        if (stroke.isMergingBoundary) {
            return stroke.points;
        }
    }
    return {};
}

MeshPart monster::createFrontBack(const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2,
                         const std::vector<bool>& isDirichletIn,
                         int depthOrder,
                         const std::vector<bool>& isMergingIn) {
    // Mark the first n vertices as Dirichlet — these are the original boundary
    // points (Dp) that will be pinned to z=0 in the Poisson solve
    std::vector<bool> isDirichlet = isDirichletIn;

    // Pre-allocate global vertex list for worst case (all vertices duplicated)
    Eigen::MatrixXd V_global(V2.rows() * 2, 2);
    Eigen::VectorXi sideFlags(V2.rows() * 2);

    // Add front vertices (s=+1), indices 0..V2.rows()-1
    for (int i = 0; i < V2.rows(); i++) {
        V_global(i, 0) = V2(i, 0);
        V_global(i, 1) = V2(i, 1);
        sideFlags(i) = 1;
    }

    // Add back vertices (s=-1) — all get new indices since we're not sharing
    // Dirichlet vertices anymore (pinning handles the z=0 connection instead)
    int backOffset = V2.rows();
    std::vector<int> backIndexMap(V2.rows());
    for (int i = 0; i < V2.rows(); i++) {
        backIndexMap[i] = backOffset;
        V_global(backOffset, 0) = V2(i, 0);
        V_global(backOffset, 1) = V2(i, 1);
        sideFlags(backOffset) = -1;
        backOffset++;
    }

    // Trim to actual size
    V_global.conservativeResize(backOffset, 2);
    sideFlags.conservativeResize(backOffset);

    // Reverse triangle winding for back faces so normals point outward
    Eigen::MatrixXi F2_reversed = F2.rowwise().reverse();

    // Remap back face indices to the new back vertex indices
    Eigen::MatrixXi F_back_remapped(F2_reversed.rows(), 3);
    for (int i = 0; i < F2_reversed.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            F_back_remapped(i, j) = backIndexMap[F2_reversed(i, j)];
        }
    }

    // Combine front and back faces into one face list
    Eigen::MatrixXi F_global(F2.rows() + F2_reversed.rows(), 3);
    F_global << F2, F_back_remapped;

    // Remove degenerate faces (where two or more vertices share the same index)
    Eigen::MatrixXi F_clean(F_global.rows(), 3);
    int validFaces = 0;
    for (int i = 0; i < F_global.rows(); i++) {
        int a = F_global(i,0), b = F_global(i,1), c = F_global(i,2);
        if (a != b && b != c && a != c) {
            F_clean.row(validFaces++) = F_global.row(i);
        }
    }
    F_clean.conservativeResize(validFaces, 3);
    F_global = F_clean;

    // Build and return the MeshPart
    MeshPart part;
    part.V = V_global;
    part.F = F_global;
    part.sideFlags = sideFlags;
    part.depthOrder = depthOrder;

    const int totalV = V_global.rows();
    const int frontCount = V2.rows();
    part.isDirichlet.resize(totalV, false);
    part.isMerging.resize(totalV, false);

    for (int i = 0; i < totalV; i++) {
        bool isFront = (i < frontCount);
        int srcIdx = isFront ? i : (i - frontCount);
        // Dirichlet: first n vertices are on Dp
        part.isDirichlet[i] = isDirichletIn[srcIdx] && !isMergingIn[srcIdx];
        // Merging: propagate from input, for both front and back copies
        if (!isMergingIn.empty())
            part.isMerging[i] = isMergingIn[srcIdx];
    }

    return part;
}

std::vector<int> monster::splitAlongBp(Eigen::MatrixXd& V2, Eigen::MatrixXi& F2,
                           const std::vector<Eigen::Vector2f>& bpPoints) {
    if (bpPoints.size() < 2) return {};

    // Step 1: Find Bp vertex indices in V2
    std::vector<int> bpIndices;
    for (const Eigen::Vector2f& bp : bpPoints) {
        for (int i = 0; i < V2.rows(); i++) {
            double dx = V2(i,0) - bp.x();
            double dy = V2(i,1) - bp.y();
            if (dx*dx + dy*dy < 0.01) {
                bpIndices.push_back(i);
                break;
            }
        }
    }
    std::cout << "Bp indices found: " << bpIndices.size() << std::endl;
    if (bpIndices.size() < 2) return {};

    // p and q are the two endpoints of Bp
    Eigen::Vector2d p = V2.row(bpIndices[0]);
    Eigen::Vector2d q = V2.row(bpIndices[1]);
    Eigen::Vector2d edge = q - p;

    // Step 2: Duplicate Bp vertices (skip first and last — they sit on Dp
    // and are shared by the silhouette boundary, duplicating them causes fans)
    int nOld = V2.rows();
    std::vector<int> interiorBpIndices;
    for (int i = 1; i < (int)bpIndices.size() - 1; i++)  // skip 0 and last
        interiorBpIndices.push_back(bpIndices[i]);

    V2.conservativeResize(nOld + interiorBpIndices.size(), 2);
    std::vector<int> duplicateIndices;
    for (int i = 0; i < (int)interiorBpIndices.size(); i++) {
        V2.row(nOld + i) = V2.row(interiorBpIndices[i]);
        duplicateIndices.push_back(nOld + i);
    }

    // Step 3: Classify triangles and remap (use interiorBpIndices not bpIndices)
    for (int t = 0; t < F2.rows(); t++) {
        bool touchesBp = false;
        for (int j = 0; j < 3; j++)
            for (int bi : interiorBpIndices)
                if (F2(t,j) == bi) { touchesBp = true; break; }
        if (!touchesBp) continue;

        // Compute triangle centroid
        Eigen::Vector2d c = (V2.row(F2(t,0)) + V2.row(F2(t,1)) + V2.row(F2(t,2))) / 3.0;

        // Classify using cross product: cross(edge, c - p)
        double cross = edge.x() * (c.y() - p.y()) - edge.y() * (c.x() - p.x());

        if (cross > 0) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < (int)bpIndices.size(); k++) {
                    if (F2(t,j) == bpIndices[k]) {
                        F2(t,j) = duplicateIndices[k];
                    }
                }
            }
        }
    }

    std::cout << "Split along Bp: " << bpIndices.size()
              << " vertices duplicated" << std::endl;
    // return the endpoint indices (the armpits)
    return { bpIndices.front(), bpIndices.back() };
}

void monster::triangulateRegion(const Region& region, Eigen::MatrixXd& V, int& n,
                                Eigen::MatrixXd& V2, Eigen::MatrixXi& F2,
                                const std::vector<Eigen::Vector2f>& extraPoints) {
    std::vector<Eigen::Vector2f> boundaryPoints;
    for (const Stroke& stroke : region.boundaries) {
        int m = stroke.points.size();
        for (int i = 0; i < m - 1; i++) {
            boundaryPoints.push_back(stroke.points[i]);
        }
    }
    n = boundaryPoints.size();
    int totalPoints = n + (int)extraPoints.size();
    V.resize(totalPoints, 2);
    for (int i = 0; i < n; i++) {
        V(i, 0) = boundaryPoints[i].x();
        V(i, 1) = boundaryPoints[i].y();
    }
    for (int i = 0; i < (int)extraPoints.size(); i++) {
        V(n + i, 0) = extraPoints[i].x();
        V(n + i, 1) = extraPoints[i].y();
    }
    std::vector<Eigen::Vector2i> edges;
    for (int i = 0; i < n - 1; i++) {
        edges.push_back({i, i + 1});
    }
    edges.push_back({n - 1, 0});
    if (extraPoints.size() == 2) {
        edges.push_back({n, n + 1});
    }
    Eigen::MatrixXi E(edges.size(), 2);
    for (int i = 0; i < (int)edges.size(); i++) {
        E(i, 0) = edges[i].x();
        E(i, 1) = edges[i].y();
    }
    Eigen::MatrixXd H(0, 2);
    igl::triangle::triangulate(V, E, H, "pQa100q20", V2, F2);
}

void monster::triangulateRegions(
    const Region& host,
    const std::vector<const Region*>& attachments,
    Eigen::MatrixXd& V2,
    Eigen::MatrixXi& F2,
    std::vector<bool>& isDirichlet,
    std::vector<bool>& isMerging,
    std::vector<std::pair<int,int>>& armpitPairs)
{
    // =========================================================
    // Phase 1: Collect host boundary points
    // =========================================================
    std::vector<Eigen::Vector2f> hostBoundary;
    for (const Stroke& stroke : host.boundaries) {
        int m = stroke.points.size();
        for (int i = 0; i < m - 1; i++)
            hostBoundary.push_back(stroke.points[i]);
    }
    int nHost = hostBoundary.size();

    // =========================================================
    // Phase 2: Collect Bp points for each attachment
    // =========================================================
    // bpByAttachment[i] = {bp0, bp1} for attachment i
    std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> bpByAttachment;
    for (const Region* att : attachments) {
        auto pts = getMergingBoundaryPoints(*att);
        if (pts.size() >= 2)
            bpByAttachment.push_back({pts.front(), pts.back()});
        else
            bpByAttachment.push_back({{0,0},{0,0}}); // degenerate, skip
    }

    // =========================================================
    // Phase 3: Build combined vertex list
    // =========================================================
    // Host boundary first, then Bp interior points
    // Bp endpoints are already in the host boundary (they lie on Dp)
    // so we don't add them again — we just find their indices
    std::vector<Eigen::Vector2f> allVerts;
    for (auto& p : hostBoundary) allVerts.push_back(p);

    // Find indices of Bp endpoints in host boundary
    auto findOrAdd = [&](const Eigen::Vector2f& p) -> int {
        for (int i = 0; i < (int)allVerts.size(); i++) {
            if ((allVerts[i] - p).norm() < 0.5f) return i;
        }
        allVerts.push_back(p);
        return allVerts.size() - 1;
    };

    // For each attachment, find/add its Bp endpoints
    std::vector<std::pair<int,int>> bpEndpointIndices; // per attachment
    for (auto& bp : bpByAttachment) {
        int i0 = findOrAdd(bp.first);
        int i1 = findOrAdd(bp.second);
        bpEndpointIndices.push_back({i0, i1});
    }

    // =========================================================
    // Phase 4: Build edge list
    // =========================================================
    std::vector<Eigen::Vector2i> edges;

    // Host boundary loop — but split at Bp endpoints
    // We need to ensure Bp endpoints are proper vertices in the boundary
    // Rebuild boundary loop inserting Bp endpoints where they fall
    // For simplicity: boundary is already sampled densely enough that
    // Bp endpoints are exact matches — findOrAdd handles this above
    for (int i = 0; i < nHost - 1; i++)
        edges.push_back({i, i + 1});
    edges.push_back({nHost - 1, 0}); // close loop

    // Bp constraint edges
    for (auto& ep : bpEndpointIndices)
        edges.push_back({ep.first, ep.second});

    // =========================================================
    // Phase 5: Build Eigen matrices and triangulate
    // =========================================================
    int nVerts = allVerts.size();
    Eigen::MatrixXd V(nVerts, 2);
    for (int i = 0; i < nVerts; i++) {
        V(i, 0) = allVerts[i].x();
        V(i, 1) = allVerts[i].y();
    }
    Eigen::MatrixXi E(edges.size(), 2);
    for (int i = 0; i < (int)edges.size(); i++) {
        E(i, 0) = edges[i].x();
        E(i, 1) = edges[i].y();
    }
    Eigen::MatrixXd H(0, 2);
    igl::triangle::triangulate(V, E, H, "pQa100q20", V2, F2);

    // =========================================================
    // Phase 6: Classify output vertices
    // =========================================================
    int nOut = V2.rows();
    isDirichlet.assign(nOut, false);
    isMerging.assign(nOut, false);

    // Mark Dirichlet: vertices that match host boundary points (not Bp endpoints)
    // In the vertex classification loop in triangulateRegions:
    for (int i = 0; i < nOut; i++) {
        Eigen::Vector2d p = V2.row(i);
        // check against all Bp segments
        for (int bi = 0; bi < (int)bpByAttachment.size(); bi++) {
            auto& bp = bpByAttachment[bi];
            Eigen::Vector2d bp0 = bp.first.cast<double>();
            Eigen::Vector2d bp1 = bp.second.cast<double>();
            Eigen::Vector2d seg = bp1 - bp0;
            double t = (p - bp0).dot(seg) / seg.dot(seg);
            t = std::max(0.0, std::min(1.0, t));
            double dist = (p - (bp0 + t * seg)).norm();
            if (dist < 0.5) {
                // check if it's an endpoint (armpit) — mark differently
                bool isEndpoint = (p - bp0).norm() < 0.5 ||
                                  (p - bp1).norm() < 0.5;
                if (!isEndpoint) {
                    isMerging[i] = true;
                    isDirichlet[i] = false;
                }
                // armpit endpoints: neither merging nor dirichlet
                // they stay as false/false — handled separately via armpitPairs
                break;
            }
        }
    }

    // Host outer contour Dp: Dirichlet where the vertex lies on the closed
    // host-boundary polyline. Bp interior stays excluded via isMerging; Bp
    // endpoints lie on Dp and are Dirichlet like the rest of the silhouette.
    auto distPointToSegment = [](const Eigen::Vector2d& p,
                                  const Eigen::Vector2d& a,
                                  const Eigen::Vector2d& b) -> double {
        Eigen::Vector2d ab = b - a;
        double denom = ab.squaredNorm();
        if (denom < 1e-30) return (p - a).norm();
        double t = (p - a).dot(ab) / denom;
        t = std::max(0.0, std::min(1.0, t));
        return (p - (a + t * ab)).norm();
    };
    if (nHost >= 3) {
        for (int i = 0; i < nOut; i++) {
            if (isMerging[i]) continue;
            Eigen::Vector2d p = V2.row(i);
            double best = std::numeric_limits<double>::infinity();
            for (int j = 0; j < nHost; j++) {
                Eigen::Vector2d a = hostBoundary[j].cast<double>();
                Eigen::Vector2d b = hostBoundary[(j + 1) % nHost].cast<double>();
                best = std::min(best, distPointToSegment(p, a, b));
                if (best < kContourDirichletEps) break;
            }
            if (best < kContourDirichletEps)
                isDirichlet[i] = true;
        }
    }

    // Mark armpit pairs: Bp endpoint vertices, front and back
    // (back indices come after front/back duplication in createFrontBack)
    for (auto& ep : bpEndpointIndices) {
        // find which V2 indices correspond to ep.first and ep.second
        Eigen::Vector2d p0 = V.row(ep.first);
        Eigen::Vector2d p1 = V.row(ep.second);
        int v0 = -1, v1 = -1;
        for (int i = 0; i < nOut; i++) {
            if ((V2.row(i).transpose() - p0).norm() < 0.5) v0 = i;
            if ((V2.row(i).transpose() - p1).norm() < 0.5) v1 = i;
        }
        if (v0 >= 0) armpitPairs.push_back({v0, v0 + nOut}); // front, back
        if (v1 >= 0) armpitPairs.push_back({v1, v1 + nOut});
    }
}

StitchedMesh monster::stitchParts() {
    StitchedMesh result;
    int vOffset = 0;
    for (auto& part : m_meshParts) {
        for (int i = 0; i < part.V.rows(); i++) {
            result.isDirichlet.push_back(part.isDirichlet[i]);
            result.isMerging.push_back(part.isMerging[i]);
            result.sideFlags.conservativeResize(result.sideFlags.size() + 1);
            result.sideFlags(result.sideFlags.size() - 1) = part.sideFlags(i);
        }
        int oldVRows = result.V.rows();
        result.V.conservativeResize(oldVRows + part.V.rows(), 2);
        result.V.bottomRows(part.V.rows()) = part.V;

        int oldFRows = result.F.rows();
        result.F.conservativeResize(oldFRows + part.F.rows(), 3);
        result.F.bottomRows(part.F.rows()) = part.F.array() + vOffset;

        // propagate armpit pairs with offset
        for (auto& p : part.armpitPairs)
            result.armpitPairs.push_back({p.first + vOffset, p.second + vOffset});

        vOffset += part.V.rows();
    }
    weldSeams(result);
    std::cout << "Final mesh: " << result.V.rows() << " vertices, "
              << result.F.rows() << " faces" << std::endl;
    return result;
}

void monster::weldSeams(StitchedMesh& mesh) {
    const double WELD_EPS = 0.5;
    const int n = mesh.V.rows();

    std::cout << "Armpit pair indices: ";
    for (auto& p : mesh.armpitPairs)
        std::cout << "(" << p.first << "," << p.second << ") ";
    std::cout << std::endl;

    std::vector<int> remap(n);
    std::iota(remap.begin(), remap.end(), 0);

    int weldCount = 0;

    // add here:
    for (int i = 0; i < n; i++) {
        if (!mesh.isMerging[i]) continue;
        auto key = std::make_pair(
            std::make_pair(
                (int)std::round(mesh.V(i,0) / WELD_EPS),
                (int)std::round(mesh.V(i,1) / WELD_EPS)),
            mesh.sideFlags(i));
        std::cout << "  vert " << i << " (" << mesh.V(i,0) << "," << mesh.V(i,1)
                  << ") key=(" << key.first.first << "," << key.first.second << ")" << std::endl;
    }

    // Pass 1: weld merging boundary vertices (Bp) — front-to-front, back-to-back only
    std::map<std::pair<std::pair<int,int>, int>, int> sideGrid;
    for (int i = 0; i < n; i++) {
        if (!mesh.isMerging[i]) continue;

        // skip armpit vertices — they stay as coincident but separate
        bool isArmpit = false;
        for (auto& p : mesh.armpitPairs)
            if (i == p.first || i == p.second) { isArmpit = true; break; }
        if (isArmpit) continue;
        std::cout << "  vert " << i << " isArmpit=" << isArmpit
                  << " remap=" << remap[i] << std::endl;

        auto key = std::make_pair(
            std::make_pair(
                (int)std::round(mesh.V(i,0) / WELD_EPS),
                (int)std::round(mesh.V(i,1) / WELD_EPS)),
            mesh.sideFlags(i));
        auto it = sideGrid.find(key);
        if (it != sideGrid.end()) {
            remap[i] = it->second;
            std::cout << "WELDED " << i << " -> " << it->second
                      << " at (" << mesh.V(i,0) << "," << mesh.V(i,1) << ")" << std::endl;
        } else {
            sideGrid[key] = i;
            std::cout << "ADDED  " << i << " key=(" << key.first.first
                      << "," << key.first.second << "," << key.second << ")" << std::endl;
        }
    }


    for (int i = 0; i < n; i++)
        if (remap[i] != i) weldCount++;
    std::cout << "Pass 1 welded: " << weldCount << " vertices" << std::endl;

    // Pass 2: weld Dp silhouette vertices — front matches back to close the surface
    std::map<std::pair<int,int>, int> dpGrid;
    for (int i = 0; i < n; i++) {
        if (!mesh.isDirichlet[i]) continue;
        if (mesh.isMerging[i]) continue;
        // also skip armpit vertices
        bool isArmpit = false;
        for (auto& p : mesh.armpitPairs)
            if (i == p.first || i == p.second) { isArmpit = true; break; }
        if (isArmpit) continue;
        auto key = std::make_pair(
            (int)std::round(mesh.V(i,0) / WELD_EPS),
            (int)std::round(mesh.V(i,1) / WELD_EPS));
        auto it = dpGrid.find(key);
        if (it != dpGrid.end())
            remap[i] = remap[it->second];
        else
            dpGrid[key] = i;


    }

    int weldCount2 = 0;
    for (int i = 0; i < n; i++)
        if (remap[i] != i) weldCount2++;
    std::cout << "Pass 2 welded: " << weldCount2 - weldCount << " vertices" << std::endl;

    // Apply remap to F
    for (int f = 0; f < mesh.F.rows(); f++)
        for (int c = 0; c < 3; c++)
            mesh.F(f,c) = remap[mesh.F(f,c)];

    // Compact
    int nNew = 0;
    std::vector<int> reindex(n, -1);
    for (int f = 0; f < mesh.F.rows(); f++)
        for (int c = 0; c < 3; c++)
            if (reindex[mesh.F(f,c)] == -1)
                reindex[mesh.F(f,c)] = nNew++;

    Eigen::MatrixXd newV(nNew, mesh.V.cols());
    Eigen::VectorXi newSideFlags(nNew);
    std::vector<bool> newDirichlet(nNew), newMerging(nNew);
    std::vector<std::pair<int,int>> newArmpitPairs;

    for (int i = 0; i < n; i++) {
        if (reindex[i] == -1) continue;
        newV.row(reindex[i])     = mesh.V.row(i);
        newSideFlags(reindex[i]) = mesh.sideFlags(i);
        newDirichlet[reindex[i]] = mesh.isDirichlet[i];
        newMerging[reindex[i]]   = mesh.isMerging[i];
    }

    // remap armpit pairs through both remap and reindex
    for (auto& p : mesh.armpitPairs) {
        int a = reindex[remap[p.first]];
        int b = reindex[remap[p.second]];
        if (a >= 0 && b >= 0)
            newArmpitPairs.push_back({a, b});
    }

    Eigen::MatrixXi newF = mesh.F;
    for (int f = 0; f < newF.rows(); f++)
        for (int c = 0; c < 3; c++)
            newF(f,c) = reindex[mesh.F(f,c)];

    mesh.V = newV; mesh.F = newF;
    mesh.sideFlags = newSideFlags;
    mesh.isDirichlet = newDirichlet;
    mesh.isMerging = newMerging;
    mesh.armpitPairs = newArmpitPairs;
}

std::vector<bool> monster::buildIsDirichlet(const Eigen::MatrixXd& V2,
                                   const Eigen::MatrixXd& V_input,
                                            int n, double eps) {
    std::vector<bool> isDirichlet(V2.rows(), false);
    for (int i = 0; i < V2.rows(); i++) {
        for (int j = 0; j < n; j++) {  // only check Dp points, not Bp
            double dx = V2(i,0) - V_input(j,0);
            double dy = V2(i,1) - V_input(j,1);
            if (dx*dx + dy*dy < eps*eps) {
                isDirichlet[i] = true;
                break;
            }
        }
    }

    int dCount = 0;
    for (bool b : isDirichlet) if (b) dCount++;
    std::cout << "Dirichlet verts: " << dCount << " / " << V2.rows() << std::endl;
    return isDirichlet;
}
