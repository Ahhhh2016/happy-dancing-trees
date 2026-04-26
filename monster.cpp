#include "monster.h"
#include <iostream>
#include <limits>
#include <cmath>
#include <numeric>
#include <map>
#include <set>
#include <igl/writeOBJ.h>
#include <igl/remove_unreferenced.h>
#include <igl/min_quad_with_fixed.h>

using namespace Eigen;
using namespace std;

monster::monster() {}

StitchedMesh monster::buildMesh(const std::vector<Region>& regions) {
    m_meshParts.clear();

    // Separate regions into hosts and attachments.
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

    auto densifyPolyline = [](const std::vector<Eigen::Vector2f>& poly, float spacing) {
        if (poly.size() < 2 || spacing <= 0.0f) return poly;
        std::vector<Eigen::Vector2f> out;
        out.push_back(poly.front());
        for (size_t i = 1; i < poly.size(); i++) {
            const Eigen::Vector2f a = poly[i - 1];
            const Eigen::Vector2f b = poly[i];
            const float len = (b - a).norm();
            const int steps = std::max(1, static_cast<int>(std::floor(len / spacing)));
            for (int s = 1; s <= steps; s++) {
                const float t = static_cast<float>(s) / static_cast<float>(steps);
                out.push_back(a + (b - a) * t);
            }
        }
        return out;
    };

    // Gather each limb Bp polyline independently.
    std::vector<std::vector<Eigen::Vector2f>> bpPolylines;
    std::vector<Eigen::Vector2d> limbInteriorSamples;
    std::vector<Region> preparedAttachments;
    for (const Region& limb : attachmentRegions) {
        std::vector<Eigen::Vector2f> bp = densifyPolyline(getMergingBoundaryPoints(limb), 2.0f);
        if (bp.size() < 2) continue;
        bpPolylines.push_back(bp);
        limbInteriorSamples.push_back(getLimbInteriorSample(limb));

        // Keep limb triangulation boundary in sync with densified Bp points.
        Region prepared = limb;
        for (Stroke& s : prepared.boundaries) {
            if (s.isMergingBoundary) {
                s.points = bp;
                break;
            }
        }
        preparedAttachments.push_back(prepared);
    }

    // 1-3) Body CDT with inserted Bp constraints + split along each Bp.
    for (const Region& region : hostRegions) {
        Eigen::MatrixXd V;
        Eigen::MatrixXd V2;
        Eigen::MatrixXi F2;
        int n;
        triangulateRegion(region, V, n, V2, F2, bpPolylines);

        std::vector<bool> hostMerging(V2.rows(), false);
        std::set<int> armpitEndpointSet;
        for (size_t li = 0; li < bpPolylines.size(); li++) {
            std::vector<int> outsideBp = splitAlongBp(V2, F2, bpPolylines[li], limbInteriorSamples[li]);
            if ((int)hostMerging.size() < V2.rows()) hostMerging.resize(V2.rows(), false);
            for (int idx : outsideBp) {
                if (idx >= 0 && idx < V2.rows()) hostMerging[idx] = true;
            }
            if (outsideBp.size() >= 2) {
                armpitEndpointSet.insert(outsideBp.front());
                armpitEndpointSet.insert(outsideBp.back());
            }
        }

        auto isDirichlet = buildIsDirichlet(V2, V, n);
        MeshPart part = createFrontBack(V2, F2, isDirichlet, region.depthOrder, hostMerging);
        for (int idx : armpitEndpointSet) {
            if (idx >= 0 && idx < V2.rows())
                part.armpitPairs.push_back({idx, idx + (int)V2.rows()});
        }
        m_meshParts.push_back(part);

        int mCount = 0; for (bool b : hostMerging) if (b) mCount++;
        std::cout << "Host V2=" << V2.rows() << " F2=" << F2.rows()
                  << " merging(outside Bp)=" << mCount << std::endl;
    }

    // 4) Leg CDT using D_p U B_p.
    for (size_t li = 0; li < preparedAttachments.size(); li++) {
        const Region& region = preparedAttachments[li];
        Eigen::MatrixXd V;
        Eigen::MatrixXd V2;
        Eigen::MatrixXi F2;
        int n;
        triangulateRegion(region, V, n, V2, F2);
        auto isDirichlet = buildIsDirichlet(V2, V, n, 0.1);
        auto isMerging = buildIsMerging(V2, bpPolylines[li], 0.5);
        m_meshParts.push_back(createFrontBack(V2, F2, isDirichlet, region.depthOrder, isMerging));

        int mCount = 0; for (bool b : isMerging) if (b) mCount++;
        std::cout << "Limb V2=" << V2.rows() << " F2=" << F2.rows()
                  << " merging(Bp)=" << mCount << std::endl;
    }

    // 5) Stitch host outside-Bp to limb Bp by same-side seam welding.
    StitchedMesh result = stitchParts();

    std::vector<bool> isFront(result.V.rows(), false);
    for (int i = 0; i < result.V.rows(); ++i) {
        isFront[i] = (result.sideFlags(i) > 0);
    }
    inflateMesh(result.V, result.F, isFront, result.isDirichlet, /*c=*/1.0);

    Eigen::MatrixXd V3D(result.V.rows(), 3);
    if (result.V.cols() >= 3) {
        V3D = result.V.leftCols(3);
    } else {
        V3D << result.V, Eigen::VectorXd::Zero(result.V.rows());
    }
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

Eigen::Vector2d monster::getLimbInteriorSample(const Region& region) const {
    for (const Stroke& stroke : region.boundaries) {
        if (stroke.isMergingBoundary) continue;
        const int m = static_cast<int>(stroke.points.size());
        if (m >= 1) return stroke.points[m / 2].cast<double>();
    }
    for (const Stroke& stroke : region.boundaries) {
        if (!stroke.points.empty()) return stroke.points.front().cast<double>();
    }
    return Eigen::Vector2d::Zero();
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
                           const std::vector<Eigen::Vector2f>& bpPolyline,
                           const Eigen::Vector2d& limbInteriorSample) {
    if (bpPolyline.size() < 2) return {};

    constexpr double kMatchEps2 = 1e-2; // (0.1)^2
    std::vector<int> bpIndices;
    bpIndices.reserve(bpPolyline.size());
    for (const Eigen::Vector2f& bp : bpPolyline) {
        int found = -1;
        for (int i = 0; i < V2.rows(); i++) {
            const double dx = V2(i,0) - bp.x();
            const double dy = V2(i,1) - bp.y();
            if (dx*dx + dy*dy < kMatchEps2) { found = i; break; }
        }
        if (found >= 0) bpIndices.push_back(found);
    }
    if (bpIndices.size() < 2) return {};

    // Duplicate every Bp vertex.
    const int nOld = V2.rows();
    V2.conservativeResize(nOld + static_cast<int>(bpIndices.size()), 2);
    std::map<int,int> dupMap; // original -> duplicate
    for (int k = 0; k < static_cast<int>(bpIndices.size()); k++) {
        const int orig = bpIndices[k];
        const int dup = nOld + k;
        V2.row(dup) = V2.row(orig);
        dupMap[orig] = dup;
    }

    auto sideOfPolyline = [&](const Eigen::Vector2d& x) {
        // Signed side against the closest Bp segment.
        double bestAbs = std::numeric_limits<double>::infinity();
        double bestSide = 0.0;
        for (size_t i = 0; i + 1 < bpPolyline.size(); i++) {
            const Eigen::Vector2d a = bpPolyline[i].cast<double>();
            const Eigen::Vector2d b = bpPolyline[i + 1].cast<double>();
            const Eigen::Vector2d ab = b - a;
            const double ab2 = ab.dot(ab);
            if (ab2 < 1e-12) continue;
            double t = (x - a).dot(ab) / ab2;
            t = std::max(0.0, std::min(1.0, t));
            const Eigen::Vector2d proj = a + t * ab;
            const double s = ab.x() * (x.y() - a.y()) - ab.y() * (x.x() - a.x());
            const double dist = (x - proj).norm();
            if (dist < bestAbs) {
                bestAbs = dist;
                bestSide = s;
            }
        }
        return bestSide;
    };

    const double insideSign = sideOfPolyline(limbInteriorSample);

    // Triangles on the limb side get remapped to duplicated Bp vertices.
    for (int t = 0; t < F2.rows(); t++) {
        bool touchesBp = false;
        for (int c = 0; c < 3; c++) {
            if (dupMap.find(F2(t,c)) != dupMap.end()) { touchesBp = true; break; }
        }
        if (!touchesBp) continue;

        const Eigen::Vector2d centroid =
            (V2.row(F2(t,0)) + V2.row(F2(t,1)) + V2.row(F2(t,2))) / 3.0;
        const double triSign = sideOfPolyline(centroid);
        if (triSign * insideSign <= 0.0) continue;

        for (int c = 0; c < 3; c++) {
            auto it = dupMap.find(F2(t,c));
            if (it != dupMap.end()) F2(t,c) = it->second;
        }
    }

    // Outside copy = original indices; limb will stitch to these.
    return bpIndices;
}

void monster::triangulateRegion(const Region& region, Eigen::MatrixXd& V, int& n,
                                Eigen::MatrixXd& V2, Eigen::MatrixXi& F2,
                                const std::vector<std::vector<Eigen::Vector2f>>& interiorPolylines) {
    std::vector<Eigen::Vector2f> boundaryPoints;
    for (const Stroke& stroke : region.boundaries) {
        int m = stroke.points.size();
        for (int i = 0; i < m - 1; i++) {
            boundaryPoints.push_back(stroke.points[i]);
        }
    }

    n = boundaryPoints.size();
    std::vector<Eigen::Vector2f> allPoints = boundaryPoints;
    std::vector<std::vector<int>> interiorIndices;
    interiorIndices.reserve(interiorPolylines.size());
    for (const auto& poly : interiorPolylines) {
        std::vector<int> idx;
        idx.reserve(poly.size());
        for (const auto& p : poly) {
            idx.push_back(static_cast<int>(allPoints.size()));
            allPoints.push_back(p);
        }
        interiorIndices.push_back(std::move(idx));
    }
    V.resize(static_cast<int>(allPoints.size()), 2);

    // Add boundary points
    for (int i = 0; i < static_cast<int>(allPoints.size()); i++) {
        V(i, 0) = allPoints[i].x();
        V(i, 1) = allPoints[i].y();
    }

    // Build boundary edges (closed loop of boundary points only)
    std::vector<Eigen::Vector2i> edges;
    for (int i = 0; i < n - 1; i++) {
        edges.push_back({i, i + 1});
    }
    edges.push_back({n - 1, 0});  // close boundary

    // Add each Bp polyline as constrained interior segments.
    for (const auto& idx : interiorIndices) {
        for (size_t i = 0; i + 1 < idx.size(); i++) {
            edges.push_back({idx[i], idx[i + 1]});
        }
    }

    Eigen::MatrixXi E(edges.size(), 2);
    for (int i = 0; i < (int)edges.size(); i++) {
        E(i, 0) = edges[i].x();
        E(i, 1) = edges[i].y();
    }

    Eigen::MatrixXd H(0, 2);
    // Y: prevent Steiner points on constrained segments.
    igl::triangle::triangulate(V, E, H, "pYQa100q20", V2, F2);
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

    // Pass 3: seal slit corners by welding armpit front/back endpoint pairs.
    // This closes the small corner holes without collapsing the full seam.
    for (const auto& p : mesh.armpitPairs) {
        if (p.first >= 0 && p.first < n && p.second >= 0 && p.second < n)
            remap[p.second] = remap[p.first];
    }

    // Normalize remap chains (A->B, B->C => A->C).
    auto findRoot = [&](int x) {
        while (remap[x] != x) {
            remap[x] = remap[remap[x]];
            x = remap[x];
        }
        return x;
    };
    for (int i = 0; i < n; i++) remap[i] = findRoot(i);

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

Eigen::SparseMatrix<double> monster::buildCotangentLaplacian(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
    const int n = V.rows(); 
    std::vector<Eigen::Triplet<double>> triplets;
    std::vector<double> diagonal(n, 0.0); 

    auto cotangent = [&](int a, int b, int c) {
        Eigen::Vector3d va(V(a,0), V(a,1), 0.0); 
        Eigen::Vector3d vb(V(b,0), V(b,1), 0.0); 
        Eigen::Vector3d vc(V(c,0), V(c,1), 0.0); 
        Eigen::Vector3d u = va - vc; 
        Eigen::Vector3d v = vb - vc; 
        double area2 = u.cross(v).norm(); 
        if (area2 < 1e-12) return 0.0; // guard against degenerate triangles
        return u.dot(v) / area2; // cot(angle at c)
    };

    for (int f = 0; f < F.rows(); f++) {
        int i = F(f,0);
        int j = F(f,1);
        int k = F(f,2);
        double cij = 0.5 * cotangent(i,j,k); 
        double cjk = 0.5 * cotangent(j,k,i); 
        double cki = 0.5 * cotangent(k,i,j); 
        
        auto addEdge = [&](int a, int b, double w) {
            if (w == 0.0) return; 
            triplets.emplace_back(a, b, -w);
            triplets.emplace_back(b, a, -w);
            diagonal[a] += w;
            diagonal[b] += w;
        };

        addEdge(i,j, cij);
        addEdge(j,k, cjk);
        addEdge(k,i, cki);
    }

    for (int i = 0; i < n; ++i) triplets.emplace_back(i, i, diagonal[i]);

    Eigen::SparseMatrix<double> L(n, n);
    L.setFromTriplets(triplets.begin(), triplets.end());
    return L;
} 

Eigen::VectorXd monster::buildMass(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
    Eigen::VectorXd a = Eigen::VectorXd::Zero(V.rows());
    for (int f = 0; f < F.rows(); ++f) {
        int i = F(f,0), j = F(f,1), k = F(f,2);
        Eigen::Vector3d vi(V(i,0), V(i,1), 0.0);
        Eigen::Vector3d vj(V(j,0), V(j,1), 0.0);
        Eigen::Vector3d vk(V(k,0), V(k,1), 0.0);
        double area = 0.5 * (vj - vi).cross(vk - vi).norm();
        a(i) += area / 3.0;
        a(j) += area / 3.0;
        a(k) += area / 3.0;
    }
    return a;
}

Eigen::VectorXd monster::buildRHS(const Eigen::VectorXd& a, const std::vector<bool>& isFront, double c) {
    const int n = a.size(); 
    Eigen::VectorXd rhs(n); 
    for (int i = 0; i < n; ++i) {
        double s_i = isFront[i] ? 1.0 : -1.0;
        rhs(i) = s_i * a(i) * c; 
    }
    return rhs;
}

Eigen::VectorXd monster::solvePoisson(const Eigen::SparseMatrix<double>& L, const Eigen::VectorXd& rhs, const std::vector<bool>& isDirichlet) {
    std::vector<int> fixed_idx;
    for (int i = 0; i < (int)isDirichlet.size(); ++i)
        if (isDirichlet[i]) fixed_idx.push_back(i);

    Eigen::VectorXi b = Eigen::Map<Eigen::VectorXi>(
        fixed_idx.data(), fixed_idx.size());
    Eigen::VectorXd bc = Eigen::VectorXd::Zero(b.size());

    Eigen::SparseMatrix<double> Aeq;  // no extra equality constraints
    Eigen::VectorXd Beq;

    Eigen::VectorXd h_tilde;
    igl::min_quad_with_fixed(L, rhs, b, bc, Aeq, Beq, /*pd=*/true, h_tilde);
    return h_tilde;
}

Eigen::VectorXd monster::toSemiElliptical(const Eigen::VectorXd& h_tilde, const std::vector<bool>& isFront) {
    const int n = h_tilde.size();
    Eigen::VectorXd h0(n);
    for (int i = 0; i < n; ++i) {
        double s_i = isFront[i] ? +1.0 : -1.0;
        h0(i) = s_i * std::sqrt(std::abs(h_tilde(i)));
    }
    return h0;
}

void monster::inflateMesh(
    Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const std::vector<bool>& isFront,
    const std::vector<bool>& isDirichlet,
    double c)
{
    if (V.cols() < 3) {
        Eigen::MatrixXd V3(V.rows(), 3);
        V3 << V, Eigen::VectorXd::Zero(V.rows());
        V = V3;
    }
    auto L       = buildCotangentLaplacian(V, F);
    auto a       = buildMass(V, F);
    auto rhs     = buildRHS(a, isFront, c);
    auto h_tilde = solvePoisson(L, rhs, isDirichlet);
    auto h0      = toSemiElliptical(h_tilde, isFront);
    for (int i = 0; i < V.rows(); ++i) V(i, 2) = h0(i); 
}