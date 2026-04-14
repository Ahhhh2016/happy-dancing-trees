#include "monster.h"
#include <iostream>

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
        // get all boundary points from Region
        std::vector<Eigen::Vector2f> boundaryPoints;
        for (const Stroke& stroke : region.boundaries) {
            int m = stroke.points.size();
            for (int i = 0; i < m - 1; i++) {
                boundaryPoints.push_back(stroke.points[i]);
            }
        }

        // Build V
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

        igl::triangle::triangulate(V, E, H, "pQa500", V2, F2);

        std::cout << "Region " << region.depthOrder << ": "
                  << V2.rows() << " vertices, "
                  << F2.rows() << " triangles" << std::endl;

        std::cout << "Last E: " << E(E.rows()-1, 0) << ", " << E(E.rows()-1, 1) << std::endl;

        std::vector<bool> isDirichlet(V2.rows(), false);
        for (int i = 0; i < V.rows(); i++) {  // V.rows() = original boundary points
            isDirichlet[i] = true;
        }

        int dirichletCount = std::count(isDirichlet.begin(), isDirichlet.end(), true);
        std::cout << "Dirichlet vertices: " << dirichletCount << std::endl;

        MeshPart front;
        front.V = V2;
        front.F = F2;
        front.side = 1;
        front.depthOrder = region.depthOrder;

        MeshPart back;
        back.V = V2;
        back.F = F2.rowwise().reverse();  // flip winding
        back.side = -1;
        back.depthOrder = region.depthOrder;

        front.isDirichlet = isDirichlet;
        back.isDirichlet = isDirichlet;

        std::cout << "Front: " << front.F.rows() << " triangles" << std::endl;
        std::cout << "Back: " << back.F.rows() << " triangles" << std::endl;

        m_meshParts.push_back(front);
        m_meshParts.push_back(back);

        // Global vertex list
        Eigen::MatrixXd V_global(front.V.rows() + back.V.rows(), 2);
        Eigen::VectorXi sideFlags(front.V.rows() + back.V.rows());

        // Add front vertices
        for (int i = 0; i < front.V.rows(); i++) {
            V_global(i, 0) = front.V(i, 0);
            V_global(i, 1) = front.V(i, 1);
            sideFlags(i) = 1;
        }

        // Add back vertices, skipping Dirichlet (they share front indices)
        int backOffset = front.V.rows();
        std::vector<int> backIndexMap(back.V.rows());
        for (int i = 0; i < back.V.rows(); i++) {
            if (isDirichlet[i]) {
                backIndexMap[i] = i;  // share front vertex
            } else {
                backIndexMap[i] = backOffset;
                V_global(backOffset, 0) = back.V(i, 0);
                V_global(backOffset, 1) = back.V(i, 1);
                sideFlags(backOffset) = -1;
                backOffset++;
            }
        }
        V_global.conservativeResize(backOffset, 2);
        sideFlags.conservativeResize(backOffset);

        std::cout << "Global vertices: " << V_global.rows() << std::endl;

        // Remap back faces to global indices
        Eigen::MatrixXi F_back_remapped(back.F.rows(), 3);
        for (int i = 0; i < back.F.rows(); i++) {
            for (int j = 0; j < 3; j++) {
                F_back_remapped(i, j) = backIndexMap[back.F(i, j)];
            }
        }

        // Combine front and back faces
        Eigen::MatrixXi F_global(front.F.rows() + back.F.rows(), 3);
        F_global << front.F, F_back_remapped;

        std::cout << "Global faces: " << F_global.rows() << std::endl;


    }

    std::cout << "Total mesh parts: " << m_meshParts.size() << std::endl;

    StitchedMesh result;
    int vOffset = 0;
    for (auto& part : m_meshParts) {
        for (int i = 0; i < part.V.rows(); i++) {
            result.dirichlet.push_back(part.isDirichlet[i]);
            result.sideFlags.conservativeResize(result.sideFlags.size() + 1);
            result.sideFlags(result.sideFlags.size() - 1) = part.side;
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


