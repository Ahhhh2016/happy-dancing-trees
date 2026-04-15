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

    for (const Region& region : regions) {
        std::cout << "Region " << region.depthOrder << ":" << std::endl;
        for (const Stroke& stroke : region.boundaries) {
            std::cout << "  isClosingCurve: " << stroke.isClosingCurve
                      << " isMergingBoundary: " << stroke.isMergingBoundary
                      << " points: " << stroke.points.size() << std::endl;
        }
    }

    std::vector<Region> hostRegions;
    std::vector<Region> attachmentRegions;

    for (const Region& region : regions) {
        bool hasMergingBoundary = false;
        for (const Stroke& stroke : region.boundaries) {
            if (stroke.isMergingBoundary) {
                hasMergingBoundary = true;
                break;
            }
        }
        if (hasMergingBoundary) {
            attachmentRegions.push_back(region);
        } else {
            hostRegions.push_back(region);
        }
    }

    std::cout << "Host regions: " << hostRegions.size() << std::endl;
    std::cout << "Attachment regions: " << attachmentRegions.size() << std::endl;

    // Get Bp points from attachment regions
    std::vector<Eigen::Vector2f> bpPoints;
    for (const Region& region : attachmentRegions) {
        auto pts = getMergingBoundaryPoints(region);
        bpPoints.insert(bpPoints.end(), pts.begin(), pts.end());
    }

    // Step 1: Triangulate host regions with Bp points inserted
    for (const Region& region : hostRegions) {
        Eigen::MatrixXd V;
        Eigen::MatrixXd V2;
        Eigen::MatrixXi F2;
        int n;
        triangulateRegion(region, V, n, V2, F2, bpPoints);
        m_meshParts.push_back(stitchFrontBack(V2, F2, n, region.depthOrder));
    }

    // Step 2: Triangulate attachment regions and stitch to host along Bp
    for (const Region& region : attachmentRegions) {
        Eigen::MatrixXd V;
        Eigen::MatrixXd V2;
        Eigen::MatrixXi F2;
        int n;
        triangulateRegion(region, V, n, V2, F2);
        // Identify which V2 vertices are on the merging boundary Bp
        std::vector<Eigen::Vector2f> bpPoints = getMergingBoundaryPoints(region);
        std::vector<bool> isMerging(V2.rows(), false);
        for (int i = 0; i < n; i++) {  // only check original boundary vertices
            for (const Eigen::Vector2f& bp : bpPoints) {
                double dx = V2(i,0) - bp.x();
                double dy = V2(i,1) - bp.y();
                if (dx*dx + dy*dy < 1.0) {
                    isMerging[i] = true;
                    break;
                }
            }
        }

        MeshPart part = stitchFrontBack(V2, F2, n, region.depthOrder);

        // Resize isMerging to match doubled vertex count (front + back)
        isMerging.resize(part.V.rows(), false);
        // Back vertices that correspond to merging front vertices are also merging
        for (int i = 0; i < V2.rows(); i++) {
            if (isMerging[i]) {
                isMerging[V2.rows() + i] = true;  // mark corresponding back vertex too
            }
        }

        part.isMerging = isMerging;
        m_meshParts.push_back(part);

        stitchToHost(m_meshParts[0], m_meshParts[1]);
    }

    std::cout << "Total mesh parts: " << m_meshParts.size() << std::endl;

    // Step 3: Concatenate all mesh parts into one global StitchedMesh
    // with included vertex indices, side flags, and Dirichlet markers
    StitchedMesh result = stitchParts();

    Eigen::MatrixXd V3D(result.V.rows(), 3);
    V3D << result.V, Eigen::VectorXd::Zero(result.V.rows());
    igl::writeOBJ("mesh9.obj", V3D, result.F);

    return result;
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

    // Add extra points (e.g. Bp endpoints) to the boundary
    for (const Eigen::Vector2f& p : extraPoints) {
        boundaryPoints.push_back(p);
    }

    n = boundaryPoints.size();
    V.resize(n, 2);
    for (int i = 0; i < n; i++) {
        V(i, 0) = boundaryPoints[i].x();
        V(i, 1) = boundaryPoints[i].y();
    }

    std::vector<Eigen::Vector2i> edges;
    for (int i = 0; i < n - 1; i++) {
        edges.push_back({i, i + 1});
    }
    edges.push_back({n - 1, 0});

    Eigen::MatrixXi E(edges.size(), 2);
    for (int i = 0; i < edges.size(); i++) {
        E(i, 0) = edges[i].x();
        E(i, 1) = edges[i].y();
    }

    Eigen::MatrixXd H(0, 2);
    igl::triangle::triangulate(V, E, H, "pQa100q20", V2, F2);
}


MeshPart monster::stitchFrontBack(const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2, int n, int depthOrder) {
    std::vector<bool> isDirichlet(V2.rows(), false);
    for (int i = 0; i < n; i++) {
        isDirichlet[i] = true;
    }

    Eigen::MatrixXd V_global(V2.rows() * 2, 2);
    Eigen::VectorXi sideFlags(V2.rows() * 2);

    for (int i = 0; i < V2.rows(); i++) {
        V_global(i, 0) = V2(i, 0);
        V_global(i, 1) = V2(i, 1);
        sideFlags(i) = 1;
    }

    int backOffset = V2.rows();
    std::vector<int> backIndexMap(V2.rows());
    for (int i = 0; i < V2.rows(); i++) {
        backIndexMap[i] = backOffset;
        V_global(backOffset, 0) = V2(i, 0);
        V_global(backOffset, 1) = V2(i, 1);
        sideFlags(backOffset) = -1;
        backOffset++;
    }
    V_global.conservativeResize(backOffset, 2);
    sideFlags.conservativeResize(backOffset);

    Eigen::MatrixXi F2_reversed = F2.rowwise().reverse();
    Eigen::MatrixXi F_back_remapped(F2_reversed.rows(), 3);
    for (int i = 0; i < F2_reversed.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            F_back_remapped(i, j) = backIndexMap[F2_reversed(i, j)];
        }
    }

    Eigen::MatrixXi F_global(F2.rows() + F2_reversed.rows(), 3);
    F_global << F2, F_back_remapped;

    // Remove degenerate faces
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

StitchedMesh monster::stitchParts() {
    StitchedMesh result;
    int vOffset = 0;

    // First pass: assign global indices
    vOffset = 0;
    for (auto& part : m_meshParts) {
        if (part.globalIndexMap.empty()) {
            part.globalIndexMap.resize(part.V.rows());
            std::iota(part.globalIndexMap.begin(), part.globalIndexMap.end(), vOffset);
        } else {
            // Already has some entries from stitchToHost — add offset to non-merging
            for (int i = 0; i < (int)part.globalIndexMap.size(); i++) {
                if (part.globalIndexMap[i] >= 0) {
                    part.globalIndexMap[i] += vOffset;
                }
            }
        }
        vOffset += part.V.rows();
    }

    // Resolve negative (host) indices for attachments
    MeshPart& host = m_meshParts[0];
    for (int p = 1; p < (int)m_meshParts.size(); p++) {
        MeshPart& attachment = m_meshParts[p];
        for (int i = 0; i < (int)attachment.globalIndexMap.size(); i++) {
            if (attachment.globalIndexMap[i] < 0) {
                int hostLocalIdx = -(attachment.globalIndexMap[i] + 1);
                attachment.globalIndexMap[i] = host.globalIndexMap[hostLocalIdx];
            }
        }
    }

    // Second pass: build result using globalIndexMap
    for (auto& part : m_meshParts) {
        // Add vertices
        int oldVRows = result.V.rows();
        result.V.conservativeResize(oldVRows + part.V.rows(), 2);
        result.V.bottomRows(part.V.rows()) = part.V;

        for (int i = 0; i < part.V.rows(); i++) {
            result.dirichlet.push_back(part.isDirichlet[i]);
            result.sideFlags.conservativeResize(result.sideFlags.size() + 1);
            result.sideFlags(result.sideFlags.size() - 1) = part.sideFlags(i);
        }

        // Add faces using globalIndexMap
        int oldFRows = result.F.rows();
        result.F.conservativeResize(oldFRows + part.F.rows(), 3);
        for (int i = 0; i < part.F.rows(); i++) {
            for (int j = 0; j < 3; j++) {
                result.F(oldFRows + i, j) = part.globalIndexMap[part.F(i,j)];
            }
        }
    }

    return result;
}

// helper
std::vector<Eigen::Vector2f> monster::getMergingBoundaryPoints(const Region& region) {
    for (const Stroke& stroke : region.boundaries) {
        if (stroke.isMergingBoundary) {
            return stroke.points;
        }
    }
    return {};
}

void monster::stitchToHost(MeshPart& host, MeshPart& attachment) {
    attachment.globalIndexMap.resize(attachment.V.rows());
    std::iota(attachment.globalIndexMap.begin(), attachment.globalIndexMap.end(), 0);

    for (int i = 0; i < attachment.V.rows(); i++) {
        if (!attachment.isMerging[i]) continue;

        double bestDist = std::numeric_limits<double>::max();
        int bestIdx = -1;
        for (int j = 0; j < host.V.rows(); j++) {
            double dx = attachment.V(i,0) - host.V(j,0);
            double dy = attachment.V(i,1) - host.V(j,1);
            double dist = dx*dx + dy*dy;
            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = j;
            }
        }
        // Store host local index — stitchParts will resolve to global
        attachment.globalIndexMap[i] = -(bestIdx + 1);  // negative = host index
    }
}
