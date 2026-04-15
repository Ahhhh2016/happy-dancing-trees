#include "monster.h"
#include <iostream>
#include <igl/writeOBJ.h>

using namespace Eigen;
using namespace std;

monster::monster() {}

Region monster::makeTestBody() {
    Stroke boundary;
    boundary.points = {
        {-1.0,  0.0},
        {-0.7,  0.7},
        { 0.0,  1.0},
        { 0.7,  0.7},
        { 1.0,  0.0},
        { 0.7, -0.7},
        { 0.0, -1.0},
        {-0.7, -0.7},
        {-1.0f, 0.0f}, // close it
    };
    boundary.depthOrder = 1;

    Region r;
    r.boundaries = { boundary };
    r.depthOrder = 1;

    return r;
}

Region monster::makeTestLeg() {
    Stroke drawn;
    drawn.points = {
        {-0.3, -0.7},  // top left, sits on body boundary
        {-0.3, -1.5},  // bottom left
        { 0.3, -1.5},  // bottom right
        { 0.3, -0.7},  // top right, sits on body boundary
    };
    drawn.depthOrder = 0;

    Stroke closing;
    closing.points = { { 0.3f, -0.7f }, {-0.3f, -0.7f } };
    closing.isClosingCurve = true;
    closing.isMergingBoundary = true;

    Region r;
    r.boundaries = { drawn, closing };
    r.depthOrder = 0;
    return r;
}

StitchedMesh monster::buildMesh(const std::vector<Region>& regions) {
    // avoid bad memory allocation
    m_meshParts.clear();

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
        auto pts = getMergingBoundaryPoints(region);
        bpPoints.insert(bpPoints.end(), pts.begin(), pts.end());
    }

    // Step 1-4: Triangulate host regions with Bp inserted, then split along Bp

    std::vector<int> hostSplitIndices;
    MeshPart hostPart;
    for (const Region& region : hostRegions) {
        Eigen::MatrixXd V, V2;
        Eigen::MatrixXi F2;
        int n;
        triangulateRegion(region, V, n, V2, F2, bpPoints);
        hostSplitIndices = splitAlongBp(V2, F2, bpPoints);
        hostPart = stitchFrontBack(V2, F2, n, region.depthOrder);
    }

    for (const Region& region : attachmentRegions) {
        Eigen::MatrixXd V, V2;
        Eigen::MatrixXi F2;
        int n;
        triangulateRegion(region, V, n, V2, F2);
        MeshPart attachPart = stitchFrontBack(V2, F2, n, region.depthOrder);
        MeshPart combined = stitchAttachmentToHost(hostPart, attachPart, hostSplitIndices, bpPoints);
        std::cout << "stitchAttachmentToHost returned" << std::endl;
        std::cout << "combined V: " << combined.V.rows() << " F: " << combined.F.rows()
                  << " sideFlags: " << combined.sideFlags.size()
                  << " isDirichlet: " << combined.isDirichlet.size() << std::endl;
        m_meshParts.push_back(combined);
        std::cout << "pushed to m_meshParts" << std::endl;
    }

    // Stitch attachment to host along Bp
    stitchAttachmentToHost(m_meshParts[0], m_meshParts[1], hostSplitIndices, bpPoints);

    std::cout << "About to stitch parts, m_meshParts size: " << m_meshParts.size() << std::endl;

    // Step 7: Concatenate all MeshParts into one global StitchedMesh
    // TODO: inter-region stitching along Bp (connect body and limb meshes)
    StitchedMesh result = stitchParts();

    Eigen::MatrixXd V3D(result.V.rows(), 3);
    V3D << result.V, Eigen::VectorXd::Zero(result.V.rows());

    igl::writeOBJ("mesh12.obj", V3D, result.F);

    return result;
}

std::vector<Eigen::Vector2f> monster::getMergingBoundaryPoints(const Region& region) {
    for (const Stroke& stroke : region.boundaries) {
        if (stroke.isMergingBoundary) {
            return stroke.points;
        }
    }
    return {};
}

MeshPart monster::stitchFrontBack(const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2,
                                  int n, int depthOrder) {
    // Mark the first n vertices as Dirichlet — these are the original boundary
    // points (Dp) that will be pinned to z=0 in the Poisson solve
    std::vector<bool> isDirichlet(V2.rows(), false);
    for (int i = 0; i < n; i++) isDirichlet[i] = true;

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
    part.isDirichlet = std::vector<bool>(V_global.rows(), false);
    for (int i = 0; i < V_global.rows(); i++) {
        part.isDirichlet[i] = i < n;
    }
    part.depthOrder = depthOrder;
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

    // Step 2: Duplicate Bp vertices
    int nOld = V2.rows();
    V2.conservativeResize(nOld + bpIndices.size(), 2);
    std::vector<int> duplicateIndices;
    for (int i = 0; i < (int)bpIndices.size(); i++) {
        V2.row(nOld + i) = V2.row(bpIndices[i]);
        duplicateIndices.push_back(nOld + i);
    }

    // Step 3: Classify triangles and remap
    for (int t = 0; t < F2.rows(); t++) {
        bool touchesBp = false;
        for (int j = 0; j < 3; j++) {
            for (int bi : bpIndices) {
                if (F2(t,j) == bi) { touchesBp = true; break; }
            }
            if (touchesBp) break;
        }
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

    return duplicateIndices;
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
    std::cout << "stitchParts called, parts: " << m_meshParts.size() << std::endl;
    StitchedMesh result;
    int vOffset = 0;
    std::cout << "stitchParts: " << m_meshParts.size() << " parts" << std::endl;
    for (auto& part : m_meshParts) {
        std::cout << "  processing part V: " << part.V.rows()
                  << " F: " << part.F.rows() << std::endl;
        for (int i = 0; i < part.V.rows(); i++) {
            result.dirichlet.push_back(part.isDirichlet[i]);
            result.sideFlags.conservativeResize(result.sideFlags.size() + 1);
            result.sideFlags(result.sideFlags.size() - 1) = part.sideFlags(i);
        }
        int oldVRows = result.V.rows();
        result.V.conservativeResize(oldVRows + part.V.rows(), 2);
        result.V.bottomRows(part.V.rows()) = part.V;

        int oldFRows = result.F.rows();
        result.F.conservativeResize(oldFRows + part.F.rows(), 3);
        result.F.bottomRows(part.F.rows()) = part.F.array() + vOffset;

        vOffset += part.V.rows();
    }
    std::cout << "Final mesh: " << result.V.rows() << " vertices, "
              << result.F.rows() << " faces" << std::endl;
    return result;
}

MeshPart monster::stitchAttachmentToHost(MeshPart& host, MeshPart& attachment,
                                         const std::vector<int>& hostSplitIndices,
                                         const std::vector<Eigen::Vector2f>& bpPoints) {
    // Find Bp vertices in attachment
    std::vector<int> attachBpIndices;
    for (const Eigen::Vector2f& bp : bpPoints) {
        for (int i = 0; i < attachment.V.rows(); i++) {
            double dx = attachment.V(i,0) - bp.x();
            double dy = attachment.V(i,1) - bp.y();
            if (dx*dx + dy*dy < 0.01) {
                attachBpIndices.push_back(i);
                break;
            }
        }
    }

    // Build combined vertex list — host vertices first, then attachment
    // non-Bp vertices. Bp vertices in attachment get remapped to host split vertices.
    int hostV = host.V.rows();
    int attachV = attachment.V.rows();

    // Index map for attachment vertices into the combined mesh
    std::vector<int> attachIndexMap(attachV);
    std::iota(attachIndexMap.begin(), attachIndexMap.end(), hostV);  // default: append after host

    // Remap front Bp vertices to host split vertices
    for (int i = 0; i < (int)attachBpIndices.size(); i++) {
        attachIndexMap[attachBpIndices[i]] = hostSplitIndices[i];
    }

    // Remap back Bp vertices to host back split vertices
    for (int i = 0; i < (int)attachBpIndices.size(); i++) {
        int backAttach = attachBpIndices[i] + attachV / 2;
        int backHost   = hostSplitIndices[i] + hostV / 2;
        std::cout << "backAttach: " << backAttach << " attachV: " << attachV << std::endl;
        std::cout << "backHost: " << backHost << " hostV: " << hostV << std::endl;
        if (backAttach < attachV && backHost < hostV) {
            attachIndexMap[backAttach] = backHost;
        } else {
            std::cout << "WARNING: back index out of bounds!" << std::endl;
        }
    }

    // Build combined V
    MeshPart combined;
    combined.V.resize(hostV + attachV, 2);
    combined.V.topRows(hostV) = host.V;
    combined.V.bottomRows(attachV) = attachment.V;

    // Build combined sideFlags
    combined.sideFlags.resize(hostV + attachV);
    combined.sideFlags.head(hostV) = host.sideFlags;
    combined.sideFlags.tail(attachV) = attachment.sideFlags;

    // Build combined isDirichlet
    combined.isDirichlet.insert(combined.isDirichlet.end(),
                                host.isDirichlet.begin(), host.isDirichlet.end());
    combined.isDirichlet.insert(combined.isDirichlet.end(),
                                attachment.isDirichlet.begin(), attachment.isDirichlet.end());

    // Build combined F — host faces unchanged, attachment faces remapped
    int hostF = host.F.rows();
    int attachF = attachment.F.rows();
    combined.F.resize(hostF + attachF, 3);
    combined.F.topRows(hostF) = host.F;

    for (int i = 0; i < attachF; i++) {
        for (int j = 0; j < 3; j++) {
            combined.F(hostF + i, j) = attachIndexMap[attachment.F(i,j)];
        }
    }

    // Remove unreferenced vertices (the attachment's Bp vertices that were remapped)
    // Find which vertices are actually used
    std::vector<bool> used(combined.V.rows(), false);
    for (int i = 0; i < combined.F.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            used[combined.F(i,j)] = true;
        }
    }

    // Build compact index map
    std::vector<int> compactMap(combined.V.rows(), -1);
    int newIdx = 0;
    for (int i = 0; i < (int)combined.V.rows(); i++) {
        if (used[i]) compactMap[i] = newIdx++;
    }

    // Rebuild V, sideFlags, isDirichlet with only used vertices
    Eigen::MatrixXd V_compact(newIdx, 2);
    Eigen::VectorXi sideFlags_compact(newIdx);
    std::vector<bool> dirichlet_compact(newIdx);
    for (int i = 0; i < (int)combined.V.rows(); i++) {
        if (compactMap[i] != -1) {
            V_compact.row(compactMap[i]) = combined.V.row(i);
            sideFlags_compact(compactMap[i]) = combined.sideFlags(i);
            dirichlet_compact[compactMap[i]] = combined.isDirichlet[i];
        }
    }

    // Remap faces
    for (int i = 0; i < combined.F.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            combined.F(i,j) = compactMap[combined.F(i,j)];
        }
    }

    combined.V = V_compact;
    combined.sideFlags = sideFlags_compact;
    combined.isDirichlet = dirichlet_compact;

    std::cout << "After cleanup: " << combined.V.rows() << " vertices" << std::endl;

    combined.depthOrder = host.depthOrder;

    std::cout << "Combined mesh: " << combined.V.rows() << " vertices, "
              << combined.F.rows() << " faces" << std::endl;

    std::cout << "Combined F max: " << combined.F.maxCoeff() << std::endl;
    std::cout << "Combined V rows: " << combined.V.rows() << std::endl;
    std::cout << "Returning combined" << std::endl;

    return combined;
}
