#include "monster.h"
#include <iostream>
#include <igl/writeOBJ.h>
#include <igl/remove_unreferenced.h>
#include <igl/boundary_loop.h>
#include <QImage>
#include <QPainter>
#include <set>

using namespace Eigen;
using namespace std;

monster::monster() {}

StitchedMesh monster::buildMesh(const std::vector<Region>& regions,
                                const std::vector<std::vector<int>>& connectedRegions,
                                int canvasWidth, int canvasHeight) {
    std::vector<int> hostIndices, attachmentIndices;
    for (int i=0; i<(int)regions.size(); i++) {
        bool hasMerging=false;
        for (const Stroke& s : regions[i].boundaries)
            if (s.isMergingBoundary) { hasMerging=true; break; }
        if (hasMerging) attachmentIndices.push_back(i);
        else hostIndices.push_back(i);
    }
    std::cout << "Host regions: " << hostIndices.size() << std::endl;
    std::cout << "Attachment regions: " << attachmentIndices.size() << std::endl;

    auto getComponent = [&](int regionIdx) -> std::vector<int> {
        for (const auto& comp : connectedRegions)
            for (int idx : comp)
                if (idx==regionIdx) return comp;
        return {};
    };
    auto getHostForAttachment = [&](int ai) -> int {
        auto comp = getComponent(ai);
        for (int idx : comp)
            for (int hi : hostIndices)
                if (idx==hi) return hi;
        return -1;
    };

    auto downsample = [](const std::vector<Eigen::Vector2f>& pts, int maxPts) {
        if ((int)pts.size() <= maxPts) return pts;
        std::vector<Eigen::Vector2f> result;
        float step = (float)(pts.size()-1)/(maxPts-1);
        for (int i=0; i<maxPts; i++)
            result.push_back(pts[(int)(i*step)]);
        return result;
    };

    auto subtractMask = [](const QImage& host, const QImage& attachment) -> QImage {
        QImage result = host;
        for (int y=0; y<host.height(); y++)
            for (int x=0; x<host.width(); x++)
                if (qGray(attachment.pixel(x,y)) > 128)
                    result.setPixel(x, y, qRgb(0,0,0));
        return result;
    };

    auto findInteriorPoint = [](const QImage& mask) -> Eigen::Vector2d {
        int w=mask.width(), h=mask.height();
        double cx=0, cy=0; int count=0;
        for (int y=0; y<h; y++)
            for (int x=0; x<w; x++)
                if (qGray(mask.pixel(x,y)) > 128) { cx+=x; cy+=y; count++; }
        if (count>0) return {cx/count, cy/count};
        return {-1,-1};
    };

    auto findAllBoundaryVerts = [](const Eigen::MatrixXi& F2, int nOut,
                                   std::vector<bool>& isDirichlet) {
        std::map<int,int> nextBnd;
        for (int f=0; f<F2.rows(); f++) {
            for (int k=0; k<3; k++) {
                int a = F2(f,k), b = F2(f,(k+1)%3);
                bool found = false;
                for (int f2=0; f2<F2.rows() && !found; f2++)
                    for (int k2=0; k2<3 && !found; k2++)
                        if (F2(f2,k2)==b && F2(f2,(k2+1)%3)==a) found=true;
                if (!found) nextBnd[a] = b;
            }
        }
        std::set<int> visited;
        for (auto& kv : nextBnd) {
            if (visited.count(kv.first)) continue;
            int start = kv.first, cur = start;
            do {
                isDirichlet[cur] = true;
                visited.insert(cur);
                cur = nextBnd[cur];
            } while (cur != start && !visited.count(cur));
        }
    };

    // render all masks
    std::map<int,QImage> masks;
    for (int i=0; i<(int)regions.size(); i++)
        masks[i] = renderRegionToMask(regions[i], canvasWidth, canvasHeight);

    // store hole boundaries so attachments use exactly the same points
    std::map<int, std::vector<Eigen::Vector2f>> holeBoundaryByAttachment;

    // triangulate host regions
    for (int hi : hostIndices) {
        const Region& host = regions[hi];
        const QImage& hostMask = masks[hi];

        std::vector<int> hostAttachmentIndices;
        for (int ai : attachmentIndices)
            if (getHostForAttachment(ai)==hi)
                hostAttachmentIndices.push_back(ai);

        // subtract all attachments from host mask to create holes
        QImage hostWithHoles = hostMask;
        for (int ai : hostAttachmentIndices)
            hostWithHoles = subtractMask(hostWithHoles, masks[ai]);

        // trace and downsample outer boundary
        auto outerBoundary = downsample(traceBoundary(hostWithHoles), 300);
        int nOuter = outerBoundary.size();
        std::cout << "Host outer boundary: " << nOuter << " pts" << std::endl;
        if (nOuter < 3) continue;

        // trace hole boundaries — store for reuse by attachments
        std::vector<std::vector<Eigen::Vector2f>> holeBoundaries;
        std::vector<Eigen::Vector2d> holePoints;
        for (int ai : hostAttachmentIndices) {
            auto holeBoundary = downsample(traceBoundary(masks[ai]), 300);
            std::cout << "Hole " << ai << " boundary: " << holeBoundary.size() << " pts" << std::endl;
            if (holeBoundary.size() < 3) continue;
            holeBoundaryByAttachment[ai] = holeBoundary; // store for attachment loop
            holeBoundaries.push_back(holeBoundary);
            holePoints.push_back(findInteriorPoint(masks[ai]));
        }

        // build V: outer boundary + hole boundaries
        int totalVerts = nOuter;
        for (auto& hb : holeBoundaries) totalVerts += hb.size();
        Eigen::MatrixXd V(totalVerts, 2);
        for (int i=0; i<nOuter; i++) {
            V(i,0)=outerBoundary[i].x();
            V(i,1)=outerBoundary[i].y();
        }
        int offset = nOuter;
        for (auto& hb : holeBoundaries) {
            for (int i=0; i<(int)hb.size(); i++) {
                V(offset+i,0)=hb[i].x();
                V(offset+i,1)=hb[i].y();
            }
            offset += hb.size();
        }

        // build E: outer loop + hole loops
        std::vector<Eigen::Vector2i> edges;
        for (int i=0; i<nOuter-1; i++) edges.push_back({i,i+1});
        edges.push_back({nOuter-1,0});
        offset = nOuter;
        for (auto& hb : holeBoundaries) {
            int n = hb.size();
            for (int i=0; i<n-1; i++) edges.push_back({offset+i, offset+i+1});
            edges.push_back({offset+n-1, offset});
            offset += n;
        }

        Eigen::MatrixXi E(edges.size(), 2);
        for (int i=0; i<(int)edges.size(); i++) { E(i,0)=edges[i].x(); E(i,1)=edges[i].y(); }

        // H: one point inside each hole
        Eigen::MatrixXd H(holePoints.size(), 2);
        for (int i=0; i<(int)holePoints.size(); i++) {
            H(i,0)=holePoints[i].x();
            H(i,1)=holePoints[i].y();
        }

        Eigen::MatrixXd V2; Eigen::MatrixXi F2;
        std::cout << "Triangulating host: V=" << V.rows() << " E=" << E.rows() << " H=" << H.rows() << std::endl;
        igl::triangle::triangulate(V, E, H, "pQa100q20", V2, F2);
        std::cout << "Host done: " << V2.rows() << " verts" << std::endl;

        int nOut = V2.rows();
        std::vector<bool> isDirichlet(nOut,false), isMerging(nOut,false);
        std::vector<std::pair<int,int>> armpitPairs;

        // Dirichlet = all boundary vertices
        findAllBoundaryVerts(F2, nOut, isDirichlet);

        // isMerging = Bp pixels, overrides Dirichlet
        for (int hi2=0; hi2<(int)holeBoundaries.size(); hi2++) {
            int ai = hostAttachmentIndices[hi2];
            auto bpPixels = findBpPixels(hostMask, masks[ai]);
            auto rawBp = getMergingBoundaryPoints(regions[ai]);
            Eigen::Vector2f bp0(round(rawBp.front().x()), round(rawBp.front().y()));
            Eigen::Vector2f bp1(round(rawBp.back().x()), round(rawBp.back().y()));

            for (int i=0; i<nOut; i++) {
                Eigen::Vector2d p = V2.row(i);
                for (auto& bp : bpPixels) {
                    double dx=p.x()-bp.x(), dy=p.y()-bp.y();
                    if (dx*dx+dy*dy<0.25) { isMerging[i]=true; isDirichlet[i]=false; break; }
                }
                Eigen::Vector2f pf(V2(i,0),V2(i,1));
                if ((pf-bp0).norm()<1.0f || (pf-bp1).norm()<1.0f) {
                    isDirichlet[i]=true; isMerging[i]=false;
                    armpitPairs.push_back({i, i+nOut});
                }
            }
        }

        MeshPart part = createFrontBack(V2, F2, isDirichlet, host.depthOrder, isMerging);
        for (auto& ap : armpitPairs) part.armpitPairs.push_back(ap);
        m_meshParts.push_back(part);
    }

    // triangulate attachment regions
    for (int ai : attachmentIndices) {
        const Region& region = regions[ai];

        // use the same boundary that the host used for this hole
        std::vector<Eigen::Vector2f> boundary;
        if (holeBoundaryByAttachment.count(ai))
            boundary = holeBoundaryByAttachment[ai];
        else
            boundary = downsample(traceBoundary(masks[ai]), 300);

        int nBoundary = boundary.size();
        std::cout << "Attachment " << ai << " boundary: " << nBoundary << " pts" << std::endl;
        if (nBoundary < 3) continue;

        Eigen::MatrixXd V(nBoundary, 2);
        for (int i=0; i<nBoundary; i++) { V(i,0)=boundary[i].x(); V(i,1)=boundary[i].y(); }

        std::vector<Eigen::Vector2i> edges;
        for (int i=0; i<nBoundary-1; i++) edges.push_back({i,i+1});
        edges.push_back({nBoundary-1,0});

        Eigen::MatrixXi E(edges.size(), 2);
        for (int i=0; i<(int)edges.size(); i++) { E(i,0)=edges[i].x(); E(i,1)=edges[i].y(); }
        Eigen::MatrixXd V2; Eigen::MatrixXi F2; Eigen::MatrixXd H(0,2);

        std::cout << "Triangulating attachment " << ai << ": V=" << V.rows() << " E=" << E.rows() << std::endl;
        igl::triangle::triangulate(V, E, H, "pQa100q20", V2, F2);
        std::cout << "Attachment " << ai << " done: " << V2.rows() << " verts" << std::endl;

        int nOut = V2.rows();
        std::vector<bool> isDirichlet(nOut,false), isMerging(nOut,false);

        // Dirichlet = all boundary vertices
        findAllBoundaryVerts(F2, nOut, isDirichlet);

        // isMerging = Bp pixels shared with host
        int hi = getHostForAttachment(ai);
        std::vector<Eigen::Vector2f> bpPixels;
        if (hi >= 0)
            bpPixels = findBpPixels(masks[hi], masks[ai]);

        for (int i=0; i<nOut; i++) {
            Eigen::Vector2d p = V2.row(i);
            for (auto& bp : bpPixels) {
                double dx=p.x()-bp.x(), dy=p.y()-bp.y();
                if (dx*dx+dy*dy<0.25) { isMerging[i]=true; isDirichlet[i]=false; break; }
            }
        }

        auto rawBp = getMergingBoundaryPoints(region);
        Eigen::Vector2f bp0(round(rawBp.front().x()), round(rawBp.front().y()));
        Eigen::Vector2f bp1(round(rawBp.back().x()), round(rawBp.back().y()));

        MeshPart part = createFrontBack(V2, F2, isDirichlet, region.depthOrder, isMerging);

        for (int i=0; i<nOut; i++) {
            Eigen::Vector2f p(V2(i,0),V2(i,1));
            if ((p-bp0).norm()<1.0f || (p-bp1).norm()<1.0f) {
                isDirichlet[i]=true; isMerging[i]=false;
                part.armpitPairs.push_back({i, i+nOut});
            }
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

    int pass1Count = 0, pass2Count = 0;
    for (int i = 0; i < n; i++) {
        if (remap[i] != i) {
            if (mesh.isMerging[i]) pass1Count++;
            else pass2Count++;
        }
    }
    std::cout << "Pass 1 welded: " << pass1Count << std::endl;
    std::cout << "Pass 2 welded: " << pass2Count << std::endl;

    int dirichletCount = 0;
    for (int i = 0; i < n; i++)
        if (mesh.isDirichlet[i] && !mesh.isMerging[i]) dirichletCount++;
    std::cout << "Dirichlet verts (should weld): " << dirichletCount << std::endl;

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

QImage monster::renderRegionToMask(const Region& region, int width, int height) {
    QImage mask(width, height, QImage::Format_Grayscale8);
    mask.fill(0);
    QPainter painter(&mask);
    painter.setRenderHint(QPainter::Antialiasing, false);
    const Stroke& stroke = region.boundaries.front();
    QPainterPath fillPath;
    fillPath.moveTo(stroke.points.front().x(), stroke.points.front().y());
    for (std::size_t i = 1; i < stroke.points.size(); ++i)
        fillPath.lineTo(stroke.points[i].x(), stroke.points[i].y());
    fillPath.closeSubpath();
    painter.fillPath(fillPath, Qt::white);
    painter.end();
    return mask;
}

std::vector<Eigen::Vector2f> monster::traceBoundary(const QImage& mask) {
    int w = mask.width(), h = mask.height();

    auto isWhite = [&](int x, int y) -> bool {
        if (x<0||y<0||x>=w||y>=h) return false;
        return qGray(mask.pixel(x,y)) > 128;
    };
    auto isBoundary = [&](int x, int y) -> bool {
        if (!isWhite(x,y)) return false;
        if (x==0||y==0||x==w-1||y==h-1) return true;
        for (int dy=-1; dy<=1; dy++)
            for (int dx=-1; dx<=1; dx++)
                if ((dx||dy) && !isWhite(x+dx,y+dy)) return true;
        return false;
    };

    // find start pixel
    int startX=-1, startY=-1;
    for (int y=0; y<h && startX<0; y++)
        for (int x=0; x<w && startX<0; x++)
            if (isBoundary(x,y)) { startX=x; startY=y; }

    if (startX < 0) return {};

    const int dx[] = {1,1,0,-1,-1,-1,0,1};
    const int dy[] = {0,1,1,1,0,-1,-1,-1};

    std::vector<Eigen::Vector2f> boundary;
    std::set<std::pair<int,int>> visited;
    int cx=startX, cy=startY, prevDir=0;
    do {
        if (visited.count({cx,cy})) break;
        visited.insert({cx,cy});
        boundary.push_back({(float)cx,(float)cy});
        bool found=false;
        for (int i=0; i<8; i++) {
            int dir=(prevDir+i)%8;
            int nx=cx+dx[dir], ny=cy+dy[dir];
            if (isBoundary(nx,ny) && !visited.count({nx,ny})) {
                prevDir=(dir+6)%8;
                cx=nx; cy=ny;
                found=true; break;
            }
        }
        if (!found) break;
    } while (cx!=startX||cy!=startY);

    return boundary;
}

std::vector<Eigen::Vector2f> monster::findBpPixels(
    const QImage& hostMask, const QImage& attachmentMask) {
    std::vector<Eigen::Vector2f> bp;
    int w=hostMask.width(), h=hostMask.height();
    auto isWhite=[](const QImage& img, int x, int y) -> bool {
        return qGray(img.pixel(x,y)) > 128;
    };
    for (int y=1; y<h-1; y++) {
        for (int x=1; x<w-1; x++) {
            if (!isWhite(attachmentMask,x,y)) continue;
            if (!isWhite(hostMask,x,y)) continue;
            bool onBoundary=false;
            for (int dy=-1; dy<=1&&!onBoundary; dy++)
                for (int dx=-1; dx<=1&&!onBoundary; dx++)
                    if ((dx||dy)&&!isWhite(attachmentMask,x+dx,y+dy))
                        onBoundary=true;
            if (onBoundary) bp.push_back({(float)x,(float)y});
        }
    }
    return bp;
}
