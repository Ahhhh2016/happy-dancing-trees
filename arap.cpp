#include "arap.h"
#include "graphics/meshloader.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
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
    m_gCurrent = m_p;
    m_hasArapL = false;
    m_meta = {};
    m_eqStatic.clear();
    m_layerPerPart.clear();
    initializeLMat();

    MatrixX3f all_vertices = MatrixX3f(V.size(), 3);
    for (unsigned long i = 0; i < V.size(); ++i) {
        all_vertices.row(i) = V[i];
    }
    coeffMin = all_vertices.colwise().minCoeff();
    coeffMax = all_vertices.colwise().maxCoeff();
}

void ARAP::init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax,
                vector<Vector3f> V, vector<Vector3i> T, const ArapLMetadata &meta)
{
    init(coeffMin, coeffMax, std::move(V), std::move(T));

    const int n = static_cast<int>(m_p.size());
    if (static_cast<int>(meta.partId.size()) != n ||
        static_cast<int>(meta.curveType.size()) != n) {
        std::cout << "ARAP-L disabled: per-vertex metadata size mismatch ("
                  << meta.partId.size() << " vs " << n << ")" << std::endl;
        return;
    }

    m_meta = meta;

    // Sanitize equality pairs to valid indices.
    m_eqStatic.clear();
    m_eqStatic.reserve(meta.equalityPairs.size());
    std::unordered_set<long long> seen;
    for (const auto &p : meta.equalityPairs) {
        if (p.first < 0 || p.second < 0 || p.first >= n || p.second >= n) continue;
        if (p.first == p.second) continue;
        const long long key = (static_cast<long long>(p.first) << 32) ^
                              static_cast<unsigned int>(p.second);
        if (seen.insert(key).second) m_eqStatic.push_back(p);
    }

    // Map partId -> layer index by sorting parts on their depth.
    int maxPart = -1;
    for (int p : m_meta.partId) if (p > maxPart) maxPart = p;
    if (maxPart < 0 || static_cast<int>(meta.partDepth.size()) <= maxPart) {
        std::cout << "ARAP-L disabled: missing part-depth metadata." << std::endl;
        return;
    }
    std::vector<std::pair<int,int>> depthPart;
    depthPart.reserve(maxPart + 1);
    for (int p = 0; p <= maxPart; ++p) {
        depthPart.push_back({meta.partDepth[p], p});
    }
    std::sort(depthPart.begin(), depthPart.end());
    m_layerPerPart.assign(maxPart + 1, 0);
    for (int layer = 0; layer < static_cast<int>(depthPart.size()); ++layer) {
        m_layerPerPart[depthPart[layer].second] = layer;
    }

    m_hasArapL = true;

    // Settle: relax mesh under static equality constraints so back-side leg
    // boundaries collapse onto the body hole edges before any user input.
    std::vector<Eigen::Vector3f> g = m_p;
    for (int it = 0; it < 10; ++it) {
        projectConstraints(g, /*Cgeq=*/{}, /*Cleq=*/{});
    }
    m_gCurrent = g;
    m_shape.setVertices(g);

    std::cout << "ARAP-L enabled: " << m_eqStatic.size() << " equality pairs, "
              << (maxPart + 1) << " parts." << std::endl;
}

void ARAP::initTexture(const std::vector<Eigen::Vector3f> &V,
                       const std::vector<Eigen::Vector3i> &T,
                       const std::vector<Eigen::Vector2f> &texcoordsPerCorner,
                       GLuint diffuseTexture)
{
    m_shape.init(V, T, texcoordsPerCorner, diffuseTexture);
    // Push the latest deformed positions so the textured GPU buffers reflect
    // post-settle / warm-started state instead of the rest pose.
    if (!m_gCurrent.empty() && m_gCurrent.size() == V.size()) {
        m_shape.setVertices(m_gCurrent);
    }
}

// Layering-constraint accessors used by GLWidget::applyArapConstraintsSidecar
// to load the legacy `_arap_constraints.txt` file. The current ARAP-L solve
// path uses the per-vertex `_arapl.txt` metadata (m_meta) instead, so these
// vectors are kept around for callers that still depend on them but they do
// not feed back into the deformation solve.
void ARAP::clearLayeringConstraints()
{
    m_ordering_constraints.clear();
    m_equality_constraints.clear();
}

void ARAP::addOrderingConstraint(int i, int j, OrderingConstraint::Type type)
{
    m_ordering_constraints.push_back({i, j, type});
}

void ARAP::addEqualityConstraint(int i, int j)
{
    m_equality_constraints.push_back({i, j});
}

void ARAP::clear()
{
    m_shape.destroyGL();
    m_p.clear();
    m_gCurrent.clear();
    m_neighbors.clear();
    m_lastAnchors.clear();
    m_L.resize(0, 0);
    m_hasArapL = false;
    m_meta = {};
    m_eqStatic.clear();
    m_layerPerPart.clear();
}

// Move an anchored vertex, defined by its index, to targetPosition
void ARAP::move(int vertex, Vector3f targetPosition)
{

    const std::unordered_set<int>& anchors = m_shape.getAnchors();

    if (anchors != m_lastAnchors) {
        initializeSolve();
        m_lastAnchors = anchors;
    }

    const std::vector<Eigen::Vector3f>& p = m_p;
    int n = p.size();

    // Warm-start from the last accepted pose so previously-moved anchors keep
    // their dragged position across successive move() calls. m_gCurrent is
    // refreshed at the end of every move (even without ARAP-L) so anchors that
    // were dragged earlier persist instead of snapping back to rest.
    const bool haveCurrent = (static_cast<int>(m_gCurrent.size()) == n);
    std::vector<Eigen::Vector3f> new_vertices = haveCurrent ? m_gCurrent : p;

    // Position to pin vertex i at when it is an anchor: the actively-dragged
    // anchor goes to its new target; every other anchor stays where it was
    // last placed (rest pose only as a safe fallback).
    auto pinnedPos = [&](int i) -> Eigen::Vector3f {
        if (i == vertex) return targetPosition;
        if (haveCurrent) return m_gCurrent[i];
        return p[i];
    };

    new_vertices[vertex] = targetPosition;

    std::vector<Matrix3f> R(n, Matrix3f::Identity());

    // Build dynamic inequality sets once per frame from the current pose.
    std::vector<std::pair<int,int>> Cgeq, Cleq;
    if (m_hasArapL) {
        Cgeq = buildInequalities(new_vertices, /*buildGeq=*/true);
        Cleq = buildInequalities(new_vertices, /*buildGeq=*/false);
    }

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
                b.row(i) = pinnedPos(i).cast<double>().transpose();
                continue;
            }

            Vector3d rhs = Vector3d::Zero();
            for (const auto& [j, w] : m_neighbors[i]) {

                // sum of (w_ij / 2) * (R_i + R_j) * (p_i - p_j)
                rhs += ((w / 2.f) * (R[i] + R[j]) * (p[i] - p[j])).cast<double>();
                if (anchors.count(j)) {

                    // anchored neighbor j contributes w_ij * c_j
                    rhs += (w * pinnedPos(j)).cast<double>();
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

        for (int a : anchors) new_vertices[a] = pinnedPos(a);

        // ARAP-L: enforce the three constraint sets after each global solve.
        if (m_hasArapL) {
            projectConstraints(new_vertices, Cgeq, Cleq);
            for (int a : anchors) new_vertices[a] = pinnedPos(a);
        }
    }

    m_gCurrent = new_vertices;
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

double ARAP::meanEdgeLength(const std::vector<Eigen::Vector3f> &g) const
{
    double sum = 0.0;
    int count = 0;
    for (const auto &f : m_shape.getFaces()) {
        const int a = f[0], b = f[1], c = f[2];
        if (a < 0 || b < 0 || c < 0) continue;
        if (a >= (int)g.size() || b >= (int)g.size() || c >= (int)g.size()) continue;
        sum += (g[a] - g[b]).head<2>().norm();
        sum += (g[b] - g[c]).head<2>().norm();
        sum += (g[c] - g[a]).head<2>().norm();
        count += 3;
    }
    return (count > 0) ? (sum / count) : 1.0;
}

std::vector<std::pair<int,int>>
ARAP::buildInequalities(const std::vector<Eigen::Vector3f> &g, bool buildGeq) const
{
    std::vector<std::pair<int,int>> pairs;
    if (!m_hasArapL) return pairs;
    const int n = static_cast<int>(g.size());
    if (n == 0) return pairs;

    const double h = std::max(meanEdgeLength(g), 1e-4);
    const double cell = 2.0 * h;
    const double maxDist = 2.0 * cell;
    const double maxDist2 = maxDist * maxDist;

    auto cellKey = [cell](float x, float y) {
        const int cx = static_cast<int>(std::floor(x / cell));
        const int cy = static_cast<int>(std::floor(y / cell));
        return (static_cast<long long>(cx) << 32) ^ static_cast<unsigned int>(cy);
    };

    std::unordered_map<long long, std::vector<int>> grid;
    grid.reserve(n * 2);
    for (int i = 0; i < n; ++i) {
        grid[cellKey(g[i].x(), g[i].y())].push_back(i);
    }

    for (int i = 0; i < n; ++i) {
        if (i >= (int)m_meta.curveType.size()) break;
        if (m_meta.curveType[i] != CurveType::DCurve) continue;
        const int p = m_meta.partId[i];
        if (p < 0 || p >= (int)m_layerPerPart.size()) continue;
        const int pLayer = m_layerPerPart[p];

        const int cx = static_cast<int>(std::floor(g[i].x() / cell));
        const int cy = static_cast<int>(std::floor(g[i].y() / cell));

        // Best candidate per other part (closest in xy).
        std::unordered_map<int, std::pair<int,double>> nearestPerPart;

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                const long long key =
                    (static_cast<long long>(cx + dx) << 32) ^
                    static_cast<unsigned int>(cy + dy);
                auto it = grid.find(key);
                if (it == grid.end()) continue;
                for (int j : it->second) {
                    if (j == i) continue;
                    if (j >= (int)m_meta.partId.size()) continue;
                    const int q = m_meta.partId[j];
                    if (q == p || q < 0 || q >= (int)m_layerPerPart.size()) continue;
                    const float dxp = g[j].x() - g[i].x();
                    const float dyp = g[j].y() - g[i].y();
                    const double dist2 = (double)dxp * dxp + (double)dyp * dyp;
                    if (dist2 > maxDist2) continue;
                    auto jt = nearestPerPart.find(q);
                    if (jt == nearestPerPart.end() || dist2 < jt->second.second) {
                        nearestPerPart[q] = {j, dist2};
                    }
                }
            }
        }

        for (const auto &kv : nearestPerPart) {
            const int q = kv.first;
            const int qLayer = m_layerPerPart[q];
            // Layer-adjacency pruning (required, see ARAP-L spec).
            if (std::abs(qLayer - pLayer) != 1) continue;
            const int j = kv.second.first;
            // Smaller partDepth = closer to camera = larger z. Skip ties so we
            // never produce both >= and <= for the same pair (which would
            // collapse the inequality into an equality).
            const int dp = m_meta.partDepth[p];
            const int dq = m_meta.partDepth[q];
            if (dp == dq) continue;
            const bool pInFront = dp < dq;
            if (buildGeq && pInFront)   pairs.push_back({i, j});  // z_i >= z_j
            if (!buildGeq && !pInFront) pairs.push_back({i, j});  // z_i <= z_j
        }
    }
    return pairs;
}

void ARAP::projectConstraints(std::vector<Eigen::Vector3f> &g,
                              const std::vector<std::pair<int,int>> &Cgeq,
                              const std::vector<std::pair<int,int>> &Cleq) const
{
    const int n = static_cast<int>(g.size());

    // Equality constraints: average all 3 coordinates.
    for (const auto &p : m_eqStatic) {
        if (p.first < 0 || p.second < 0 || p.first >= n || p.second >= n) continue;
        const Eigen::Vector3f avg = 0.5f * (g[p.first] + g[p.second]);
        g[p.first] = avg;
        g[p.second] = avg;
    }

    // z-inequalities: clamp on the mid-z when violated. Two passes help reduce
    // residual after equality projection touches the same vertices.
    for (int pass = 0; pass < 2; ++pass) {
        for (const auto &c : Cgeq) {
            if (c.first < 0 || c.second < 0 || c.first >= n || c.second >= n) continue;
            if (g[c.first].z() < g[c.second].z()) {
                const float z = 0.5f * (g[c.first].z() + g[c.second].z());
                g[c.first].z() = z;
                g[c.second].z() = z;
            }
        }
        for (const auto &c : Cleq) {
            if (c.first < 0 || c.second < 0 || c.first >= n || c.second >= n) continue;
            if (g[c.first].z() > g[c.second].z()) {
                const float z = 0.5f * (g[c.first].z() + g[c.second].z());
                g[c.first].z() = z;
                g[c.second].z() = z;
            }
        }
    }
}
