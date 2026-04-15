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

        // TODO: separate regions into hosts and attachments using isMergingBoundary
        // then triangulate host first, insert Bp as constrained edges,
        // split host along Bp, and connect attachment through the hole (Fig. 4)

        for (const Stroke& stroke : region.boundaries) {
            std::cout << "  isClosingCurve: " << stroke.isClosingCurve
                      << " isMergingBoundary: " << stroke.isMergingBoundary
                      << " points: " << stroke.points.size() << std::endl;
        }
    }

    // handle hosts and attachments somehow?


    // For each region, collect all boundary points from its strokes - skip the last point
    // of each stroke to avoid duplicates
    for (const Region& region : regions) {
        // Step 1: Collect boundary points from all strokes in this region,
        // skipping the last point of each stroke to avoid duplicates
        std::vector<Eigen::Vector2f> boundaryPoints;
        for (const Stroke& stroke : region.boundaries) {
            int m = stroke.points.size();
            for (int i = 0; i < m - 1; i++) {
                boundaryPoints.push_back(stroke.points[i]);
            }
        }

        // Step 2: Build V (Nx2 vertex matrix) and E (Nx2 edge matrix).
        // E connects each boundary point to the next, closing the loop at the end.
        int n = boundaryPoints.size();
        Eigen::MatrixXd V(n, 2);
        for (int i = 0; i < n; i++) {
            V(i, 0) = boundaryPoints[i].x();
            V(i, 1) = boundaryPoints[i].y();
        }

        std::vector<Eigen::Vector2i> edges;
        for (int i = 0; i < n - 1; i++) {
            edges.push_back({i, i + 1});
        }
        edges.push_back({n - 1, 0});  // close

        std::cout << "edges.size(): " << edges.size() << std::endl;

        Eigen::MatrixXi E(edges.size(), 2);
        for (int i = 0; i < edges.size(); i++) {
            E(i, 0) = edges[i].x();
            E(i, 1) = edges[i].y();
        }

        // output
        Eigen::MatrixXd V2;
        Eigen::MatrixXi F2;
        Eigen::MatrixXd H(0, 2); // H is holes and we start with no holes

        std::cout << "V rows: " << V.rows() << std::endl;
        std::cout << "E rows: " << E.rows() << std::endl;
        std::cout << "First V: " << V(0,0) << ", " << V(0,1) << std::endl;
        std::cout << "First E: " << E(0,0) << ", " << E(0,1) << std::endl;

        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                if (V(i,0) == V(j,0) && V(i,1) == V(j,1)) {
                    std::cout << "Duplicate at " << i << " and " << j << ": "
                              << V(i,0) << ", " << V(i,1) << std::endl;
                }
            }
        }

        // Step 3: Run constrained Delaunay triangulation (CDT).
        // V2 = output vertices (original boundary points + new interior points)
        // F2 = output triangles
        // "pQa200q20" = planar, quiet, max triangle area 200px²
        // a200 — max triangle area 200px²
        // q20 — minimum angle 20° for quality

        igl::triangle::triangulate(V, E, H, "pQa100q20", V2, F2);

        std::cout << "Region " << region.depthOrder << ": "
                  << V2.rows() << " vertices, "
                  << F2.rows() << " triangles" << std::endl;

        std::cout << "Last E: " << E(E.rows()-1, 0) << ", " << E(E.rows()-1, 1) << std::endl;

        // Step 4: Mark Dirichlet vertices.
        // The first V.rows() vertices in V2 are the original boundary points (Dp).
        // Step 3 (Poisson solve) pins these to h=0 — they form the seam of the shell.
        std::vector<bool> isDirichlet(V2.rows(), false);
        for (int i = 0; i < V.rows(); i++) {  // V.rows() = original boundary points
            isDirichlet[i] = true;
        }

        int dirichletCount = std::count(isDirichlet.begin(), isDirichlet.end(), true);
        std::cout << "Dirichlet vertices: " << dirichletCount << std::endl;

        // Step 5: Duplicate into front (s = +1) and back (s = -1) copies.
        // Front inflates toward viewer, back inflates away.
        // All vertices are duplicated — Dirichlet ones will both be pinned to z=0
        // by the Poisson solve, effectively connecting the front and back at the seam.
        Eigen::MatrixXd V_global(V2.rows() * 2, 2);
        Eigen::VectorXi sideFlags(V2.rows() * 2);
        // front vertices: indices 0..V2.rows()-1, sideFlags = +1
        // back vertices:  indices V2.rows()..end, sideFlags = -1

        // Add front vertices
        for (int i = 0; i < V2.rows(); i++) {
            V_global(i, 0) = V2(i, 0);
            V_global(i, 1) = V2(i, 1);
            sideFlags(i) = 1;
        }

        // Start adding back vertices after all front vertices
        int backOffset = V2.rows();

        // For each vertex in the back copy...
        std::vector<int> backIndexMap(V2.rows());
        for (int i = 0; i < V2.rows(); i++) {
            if (isDirichlet[i]) {
                // Boundary vertex (on Dp) - share the front's index
                // same position in 3D, this is the seam of the shell
                backIndexMap[i] = i;  // share front vertex
            } else {
                // Interior vertex - give it a new index in the global list
                // this vertex will get a different z after inflation
                backIndexMap[i] = backOffset;
                V_global(backOffset, 0) = V2(i, 0);
                V_global(backOffset, 1) = V2(i, 1);
                sideFlags(backOffset) = -1;
                backOffset++;
            }
        }
        // Trim V_global and sideFlags to actual size
        // (we pre-allocated for worst case of no sharing)
        V_global.conservativeResize(backOffset, 2);
        sideFlags.conservativeResize(backOffset);

        std::cout << "Global vertices: " << V_global.rows() << std::endl;

        // Step 6: Remap back face indices and combine front + back into F_global.
        // Back faces have reversed winding so normals point outward.
        // Remove degenerate faces (same vertex appearing twice in one triangle).
        Eigen::MatrixXi F2_reversed = F2.rowwise().reverse();
        Eigen::MatrixXi F_back_remapped(F2_reversed.rows(), 3);
        for (int i = 0; i < F2_reversed.rows(); i++) {
            for (int j = 0; j < 3; j++) {
                F_back_remapped(i, j) = backIndexMap[F2_reversed(i, j)];
            }
        }

        // Combine front and back faces
        Eigen::MatrixXi F_global(F2.rows() + F2_reversed.rows(), 3);
        F_global << F2, F_back_remapped;

        std::cout << "Global faces: " << F_global.rows() << std::endl;

        // Remove degenerate faces (where two or more vertices have the same index)
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
            part.isDirichlet[i] = i < V.rows();
        }
        part.depthOrder = region.depthOrder;

        // Store as one MeshPart per region
        m_meshParts.push_back(part);


    }

    // Step 7: Concatenate all MeshParts into one global StitchedMesh
    // with consistent vertex indices, side flags, and Dirichlet markers.
    // TODO: inter-region stitching along Bp (connect body and limb meshes)
    StitchedMesh result = stitchParts();

    Eigen::MatrixXd V3D(result.V.rows(), 3);
    V3D << result.V, Eigen::VectorXd::Zero(result.V.rows());
    igl::writeOBJ("mesh10.obj", V3D, result.F);

    return result;
}

StitchedMesh monster::stitchParts() {
    StitchedMesh result;
    int vOffset = 0;
    for (auto& part : m_meshParts) {
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

void monster::stitchRegions(StitchedMesh& mesh) {
    // Find vertices that are at the same position
    // and merge them by updating face indices

    int n = mesh.V.rows();
    std::vector<int> indexMap(n);
    std::iota(indexMap.begin(), indexMap.end(), 0); // identity map to start

    // For each pair of vertices, if they're at the same position, merge them
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            double dx = mesh.V(i,0) - mesh.V(j,0);
            double dy = mesh.V(i,1) - mesh.V(j,1);
            if (dx*dx + dy*dy < 4.0) { // within 2 pixels
                indexMap[j] = indexMap[i]; // j points to i
            }
        }
    }

    int mergedCount = 0;
    for (int i = 0; i < n; i++) {
        if (indexMap[i] != i) mergedCount++;
    }
    std::cout << "Merged vertices: " << mergedCount << std::endl;

    // Remap faces
    for (int i = 0; i < mesh.F.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            mesh.F(i,j) = indexMap[mesh.F(i,j)];
        }
    }

    // Remove degenerate faces
    Eigen::MatrixXi F_clean(mesh.F.rows(), 3);
    int validFaces = 0;
    for (int i = 0; i < mesh.F.rows(); i++) {
        int a = mesh.F(i,0), b = mesh.F(i,1), c = mesh.F(i,2);
        if (a != b && b != c && a != c) {
            F_clean.row(validFaces++) = mesh.F.row(i);
        }
    }
    F_clean.conservativeResize(validFaces, 3);
    mesh.F = F_clean;
    std::cout << "Valid faces: " << validFaces << std::endl;

    std::cout << "Stitched regions" << std::endl;
}





