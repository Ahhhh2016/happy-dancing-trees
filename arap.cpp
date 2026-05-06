#include "arap.h"
#include "graphics/meshloader.h"

#include <iostream>
#include <set>
#include <map>
#include <vector>

using namespace std;
using namespace Eigen;

ARAP::ARAP() {}

std::map<int, Vector3f> ARAP::getNeighbors(Eigen::Vector3f &i)
{
    int idx = -1;
    for (int v = 0; v < static_cast<int>(m_p.size()); ++v) {
        if ((m_p[v] - i).squaredNorm() < 1e-12f) {
            idx = v;
            break;
        }
    }
    if (idx < 0) {
        return {};
    }
    return getNeighborsByIndex(idx);
}

void ARAP::init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax, vector<Vector3f> V, vector<Vector3i> T)
{
    m_shape.init(V, T);
    m_p = m_shape.getVertices();
    initializeLMat();

    MatrixX3f all_vertices = MatrixX3f(V.size(), 3);
    for (unsigned long i = 0; i < V.size(); ++i) {
        all_vertices.row(i) = V[i];
    }
    coeffMin = all_vertices.colwise().minCoeff();
    coeffMax = all_vertices.colwise().maxCoeff();
}

// Move an anchored vertex, defined by its index, to targetPosition
void ARAP::move(int vertex, Vector3f targetPosition)
{

    const std::unordered_set<int>& anchors = m_shape.getAnchors();

    if (anchors != m_lastAnchors) {
        initializeSolve();
        m_lastAnchors = anchors;
    }

    std::vector<Eigen::Vector3f> new_vertices = m_p;

    // Determine the best-fit rotation transformations R for the moved points p′ from original points p.
    new_vertices[vertex] = targetPosition;

    const std::vector<Eigen::Vector3f>& p = m_p;
    int n = p.size();

    std::vector<Matrix3f> R(n, Matrix3f::Identity());


    const int num_iterations = 10;
    for (int iter = 0; iter < num_iterations; iter++) {


        for (int i = 0; i < n; i++) {
            // covariance matrix S_i = sum of w_ij * e_ij * e_ij'^T
            Matrix3f S = Matrix3f::Zero();
            for (const auto& [j, w] : m_neighbors[i]) {
                Vector3f e = p[i] - p[j]; // e_ij
                Vector3f ep = new_vertices[i] - new_vertices[j]; // e_ij'
                S += w * e * ep.transpose();
            }


            // best rotation is R_i = V_i * U_i^T
            JacobiSVD<Matrix3f> svd(S, ComputeFullU | ComputeFullV);
            Matrix3f U = svd.matrixU();
            Matrix3f V = svd.matrixV();
            Matrix3f Ri = V * U.transpose();
            if (Ri.determinant() < 0) {
                U.col(2) *= -1;
                Ri = V * U.transpose();
            }
            R[i] = Ri;
        }


        // build RHS b and solve L * p' = b
        MatrixXd b(n, 3);
        b.setZero();

        for (int i = 0; i < n; i++) {

            // b = current anchor position
            if (anchors.count(i)) {
                Vector3f ca = (i == vertex) ? targetPosition : p[i];
                b.row(i) = ca.cast<double>().transpose();
                continue;
            }

            Vector3d rhs = Vector3d::Zero();
            for (const auto& [j, w] : m_neighbors[i]) {

                // sum of (w_ij / 2) * (R_i + R_j) * (p_i - p_j)
                rhs += ((w / 2.f) * (R[i] + R[j]) * (p[i] - p[j])).cast<double>();
                if (anchors.count(j)) {

                    // anchored neighbor j contributes w_ij * c_j
                    Vector3f cj = (j == vertex) ? targetPosition : p[j];
                    rhs += (w * cj).cast<double>();
                }
            }
            b.row(i) = rhs.transpose();
        }

        // solve L * p' = b per coordinate axis
        VectorXd px = m_solver.solve(b.col(0));
        VectorXd py = m_solver.solve(b.col(1));
        VectorXd pz = m_solver.solve(b.col(2));

        // rebuild vertices
        for (int i = 0; i < n; i++)
            new_vertices[i] = Vector3f((float)px[i], (float)py[i], (float)pz[i]);

        new_vertices[vertex] = targetPosition;
        for (int a : anchors)
            if (a != vertex) new_vertices[a] = p[a];
    }

    m_shape.setVertices(new_vertices);

}


std::map<int, Vector3f> ARAP::getNeighborsByIndex(int idx) {
    std::map<int, Vector3f> neighbors;
    std::vector<Vector3f> verts = m_p;
    for (const Vector3i& f : m_shape.getFaces()) {
        for (int s = 0; s < 3; s++) {
            if (f[s] == idx) {
                neighbors[f[(s+1) % 3]] = verts[f[(s+1) % 3]];
                neighbors[f[(s+2) % 3]] = verts[f[(s+2) % 3]];
            }
        }
    }
    return neighbors;
}

void ARAP::initializeLMat() {
    int n = m_p.size();
    m_p = m_shape.getVertices();

    // Build the L matrix.
    MatrixXd L = MatrixXd::Zero(n, n);


    // Determine the one-ring neighbors of each vertex
    for (const Vector3i& f : m_shape.getFaces()) {
        // Calculate the cotangent weight w for each vertex; Fill in the L matrix entries.
        for (int s = 0; s < 3; s++) {
            int i = f[s];
            int j = f[(s + 1) % 3];
            int ki = f[(s + 2) % 3];


            // set up points for cotangent calculation
            Vector3f pk = m_p[ki];
            Vector3f pi = m_p[i];
            Vector3f pj = m_p[j];
            Vector3f u = pi - pk;
            Vector3f v = pj - pk;


            // cotangent calculation, with check for near 0 vals
            float sinA = u.cross(v).norm();
            float cot = (sinA > 1e-8f) ? u.dot(v) / sinA : 0.f;
            float w = std::max(0.f, 0.5f * cot);

            // each face contributes one cot angle per edge endpoint pair
            L(i, j) += w;
            L(j, i) += w;
        }
    }

    // store cot weights in neighborhood
    m_neighbors.assign(n, {});

    std::vector<Triplet<double>> triplets;
    triplets.reserve(n * 8);

    for (int i = 0; i < n; i++) {
        double diag = 0.0;
        for (int j = 0; j < n; j++) {
            if (L(i, j) > 0.0) {
                float w = (float)L(i, j);
                m_neighbors[i].emplace_back(j, w);
                triplets.emplace_back(i, j, -w);
                diag += w;
            }
        }
        triplets.emplace_back(i, i, diag);
    }

    // store sparse L matrix so we don't have to recalculate it
    m_L = SparseMatrix<double>(n, n);
    m_L.setFromTriplets(triplets.begin(), triplets.end());
    m_L.makeCompressed();
    initializeSolve();
}

void ARAP::initializeSolve() {
    m_p = m_shape.getVertices();
    SparseMatrix<double> L = m_L;

    const auto& anchors = m_shape.getAnchors();
    for (int a : anchors) {
        // Apply user constraints by deleting rows/columns from L.
        for (SparseMatrix<double>::InnerIterator it(L, a); it; ++it) {
            it.valueRef() = (it.row() == it.col()) ? 1.0 : 0.0;
        }
        for (int k = 0; k < L.outerSize(); ++k) {
            for (SparseMatrix<double>::InnerIterator it(L, k); it; ++it) {
                if (it.row() == a && it.col() != a) {
                    it.valueRef() = 0.0;
                }
            }
        }
    }


    // Precompute the decomposition of the L matrix.
    m_solver.compute(L);
    if (m_solver.info() != Eigen::Success)
        std::cout << "Solver Error" << std::endl;
}
