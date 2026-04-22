#include "monster.h"
#include <iostream>
#include <igl/writeOBJ.h>
#include <igl/remove_unreferenced.h>

using namespace Eigen;
using namespace std;

monster::monster() {}

StitchedMesh monster::buildMesh(const std::vector<Region>& regions) {

    // Separate regions into hosts and attachments
    std::vector<Region> hostRegions;
    std::vector<Region> attachmentRegions;
    for (const Region& region : regions) {
        bool hasMergingBoundary = false;
        for (const Stroke& stroke : region.boundaries) {
            if (stroke.isMergingBoundary) { hasMergingBoundary = true; break; }
        }
        if (hasMergingBoundary) attachmentRegions.push_back(region);
        else hostRegions.push_back(region);
    }
    std::cout << "Host regions: " << hostRegions.size() << std::endl;
    std::cout << "Attachment regions: " << attachmentRegions.size() << std::endl;

    // Get Bp points from all attachment regions
    std::vector<Eigen::Vector2f> bpPoints;
    for (const Region& region : attachmentRegions) {
        std::vector<Eigen::Vector2f> pts = getMergingBoundaryPoints(region);
        bpPoints.insert(bpPoints.end(), pts.begin(), pts.end());
    }

    for (Eigen::Vector2f point : bpPoints) {
        std::cout << "x: " << point.x() << std::endl;
        std::cout << "y: " << point.y() << std::endl;
    }

    // Step 1-4: Triangulate host regions with Bp inserted, then split along Bp
    for (const Region& region : hostRegions) {
        Eigen::MatrixXd V;
        Eigen::MatrixXd V2;
        Eigen::MatrixXi F2;
        int n;
        triangulateRegion(region, V, n, V2, F2, bpPoints);
        std::vector<int> armpitIndices = splitAlongBp(V2, F2, bpPoints);
        auto isDirichlet = buildIsDirichlet(V2, V, n, 0.1);
        auto isMerging = buildIsMerging(V2, bpPoints, 0.5);
        int overlap = 0;
        for (int i = 0; i < V2.rows(); i++)
            if (isDirichlet[i] && isMerging[i]) overlap++;
        std::cout << "Host Dirichlet+Merging overlap: " << overlap << std::endl;
        MeshPart part = createFrontBack(V2, F2, isDirichlet, region.depthOrder, isMerging);
        for (int idx : armpitIndices)
            part.armpitPairs.push_back({idx, idx + (int)V2.rows()});
        m_meshParts.push_back(part);

        std::cout << "Host n (boundary input points): " << n << std::endl;
        std::cout << "Host V2 total: " << V2.rows() << std::endl;

        int mCount = 0;
        for (bool b : isMerging) if (b) mCount++;
        std::cout << "Merging verts host: " << mCount << " / " << V2.rows() << std::endl;
    }


    // Step 5-6: Triangulate attachment regions
    for (const Region& region : attachmentRegions) {
        Eigen::MatrixXd V;
        Eigen::MatrixXd V2;
        Eigen::MatrixXi F2;
        int n;
        triangulateRegion(region, V, n, V2, F2);
        auto isDirichlet = buildIsDirichlet(V2, V, n, 0.1);  // add this
        auto isMerging = buildIsMerging(V2, bpPoints, 0.5);
        int overlap = 0;
        for (int i = 0; i < V2.rows(); i++)
            if (isDirichlet[i] && isMerging[i]) overlap++;
        std::cout << "Host Dirichlet+Merging overlap: " << overlap << std::endl;
        m_meshParts.push_back(createFrontBack(V2, F2, isDirichlet, region.depthOrder, isMerging));

        std::cout << "Attachment n (boundary input points): " << n << std::endl;
        std::cout << "Attachment V2 total: " << V2.rows() << std::endl;

        int mCount = 0;
        for (bool b : isMerging) if (b) mCount++;
        std::cout << "Merging verts attachment: " << mCount << " / " << V2.rows() << std::endl;
    }

    // Step 7: Concatenate all MeshParts into one global StitchedMesh
    // TODO: inter-region stitching along Bp (connect body and limb meshes)
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
    for (int i = 0; i < (int)bpIndices.size(); i++)  // skip 0 and last
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

    // Add boundary points
    for (int i = 0; i < n; i++) {
        V(i, 0) = boundaryPoints[i].x();
        V(i, 1) = boundaryPoints[i].y();
    }

    // Add extra points (Bp) after boundary points
    for (int i = 0; i < (int)extraPoints.size(); i++) {
        V(n + i, 0) = extraPoints[i].x();
        V(n + i, 1) = extraPoints[i].y();
    }

    // Build boundary edges (closed loop of boundary points only)
    std::vector<Eigen::Vector2i> edges;
    for (int i = 0; i < n - 1; i++) {
        edges.push_back({i, i + 1});
    }
    edges.push_back({n - 1, 0});  // close boundary

    // Add Bp as interior constrained edge (connects the two extra points)
    if (extraPoints.size() == 2) {
        edges.push_back({n, n + 1});  // Bp edge
    }

    Eigen::MatrixXi E(edges.size(), 2);
    for (int i = 0; i < (int)edges.size(); i++) {
        E(i, 0) = edges[i].x();
        E(i, 1) = edges[i].y();
    }

    Eigen::MatrixXd H(0, 2);
    igl::triangle::triangulate(V, E, H, "pQa100q20", V2, F2);
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

    std::vector<int> remap(n);
    std::iota(remap.begin(), remap.end(), 0);

    // Pass 1: weld merging boundary vertices (Bp) — front-to-front, back-to-back only
    std::map<std::pair<std::pair<int,int>, int>, int> sideGrid;
    for (int i = 0; i < n; i++) {
        if (!mesh.isMerging[i]) continue;

        // skip armpit vertices — they stay as coincident but separate
        bool isArmpit = false;
        for (auto& p : mesh.armpitPairs)
            if (i == p.first || i == p.second) { isArmpit = true; break; }
        if (isArmpit) continue;

        auto key = std::make_pair(
            std::make_pair(
                (int)std::round(mesh.V(i,0) / WELD_EPS),
                (int)std::round(mesh.V(i,1) / WELD_EPS)),
            mesh.sideFlags(i));
        auto it = sideGrid.find(key);
        if (it != sideGrid.end())
            remap[i] = it->second;
        else
            sideGrid[key] = i;
    }

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
