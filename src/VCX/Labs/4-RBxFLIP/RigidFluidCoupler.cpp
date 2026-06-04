#include "RigidFluidCoupler.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace VCX::Labs::RBxFLIP {
    namespace {
        glm::vec3 ToGlm(Eigen::Vector3f const& v) {
            return glm::vec3(v.x(), v.y(), v.z());
        }

        Eigen::Vector3f ToEigen(glm::vec3 const& v) {
            return Eigen::Vector3f(v.x, v.y, v.z);
        }

        glm::vec3 CellCenter(Simulator const& fluid, int i, int j, int k) {
            return glm::vec3(-0.5f) + (glm::vec3(i, j, k) + glm::vec3(0.5f)) * fluid.m_h;
        }

        int ClampIndex(int value, int upper) {
            return std::clamp(value, 0, upper - 1);
        }

        void SphereCellBounds(Simulator const& fluid,
                              glm::vec3 const& center,
                              float radius,
                              glm::ivec3& minCell,
                              glm::ivec3& maxCell) {
            glm::vec3 lo = (center - glm::vec3(radius) + glm::vec3(0.5f)) * fluid.m_fInvSpacing - glm::vec3(1.0f);
            glm::vec3 hi = (center + glm::vec3(radius) + glm::vec3(0.5f)) * fluid.m_fInvSpacing + glm::vec3(1.0f);

            minCell = glm::ivec3(
                ClampIndex(static_cast<int>(std::floor(lo.x)), fluid.m_iCellX),
                ClampIndex(static_cast<int>(std::floor(lo.y)), fluid.m_iCellY),
                ClampIndex(static_cast<int>(std::floor(lo.z)), fluid.m_iCellZ)
            );
            maxCell = glm::ivec3(
                ClampIndex(static_cast<int>(std::ceil(hi.x)), fluid.m_iCellX),
                ClampIndex(static_cast<int>(std::ceil(hi.y)), fluid.m_iCellY),
                ClampIndex(static_cast<int>(std::ceil(hi.z)), fluid.m_iCellZ)
            );
        }

        bool InsideSphere(glm::vec3 const& p, glm::vec3 const& center, float radius) {
            glm::vec3 d = p - center;
            return glm::dot(d, d) < radius * radius;
        }

    }

    void RigidFluidCoupler::EnforceSphereBoundary(Simulator& fluid, RigidBodyItem const& sphere) const {
        if (sphere.GetType() != RigidBodyType::Sphere)
            return;

        glm::vec3 const center = ToGlm(sphere.GetPosition());
        float const minDist = sphere.GetRadius() + fluid.m_particleRadius;
        Eigen::Vector3f const sphereV = sphere.Getv();
        Eigen::Vector3f const sphereW = sphere.Getw();

        for (int p = 0; p < static_cast<int>(fluid.m_particlePos.size()); ++p) {
            glm::vec3 dir = fluid.m_particlePos[p] - center;
            float dist2 = glm::dot(dir, dir);
            if (dist2 >= minDist * minDist)
                continue;

            float dist = std::sqrt(dist2);
            glm::vec3 n = dist > 1e-6f ? dir / dist : glm::vec3(0.0f, 1.0f, 0.0f);
            fluid.m_particlePos[p] = center + n * minDist;

            Eigen::Vector3f const r = ToEigen(fluid.m_particlePos[p] - center);
            glm::vec3 surfaceVel = ToGlm(sphereV + sphereW.cross(r));
            float inward = glm::dot(fluid.m_particleVel[p] - surfaceVel, n);
            if (inward < 0.0f)
                fluid.m_particleVel[p] -= inward * n;
        }
    }

    void RigidFluidCoupler::EnforceSphereTankBoundary(RigidBodyItem& sphere, glm::vec3 const& tankMin, glm::vec3 const& tankMax) const {
        if (sphere.GetType() != RigidBodyType::Sphere)
            return;

        Eigen::Vector3f shift = Eigen::Vector3f::Zero();
        Eigen::Vector3f dv = Eigen::Vector3f::Zero();
        Eigen::Vector3f const pos = sphere.GetPosition();
        Eigen::Vector3f const vel = sphere.Getv();
        float const radius = sphere.GetRadius();

        for (int axis = 0; axis < 3; ++axis) {
            float const lo = tankMin[axis] + radius;
            float const hi = tankMax[axis] - radius;

            if (pos[axis] < lo) {
                shift[axis] = lo - pos[axis];
                if (vel[axis] < 0.0f)
                    dv[axis] = -vel[axis];
            } else if (pos[axis] > hi) {
                shift[axis] = hi - pos[axis];
                if (vel[axis] > 0.0f)
                    dv[axis] = -vel[axis];
            }
        }

        if (shift.squaredNorm() > 0.0f)
            sphere.PositionShift(shift);
        if (dv.squaredNorm() > 0.0f)
            sphere.vShift(dv);
    }

    void RigidFluidCoupler::MarkSphereSolidCells(Simulator& fluid, RigidBodyItem const& sphere) const {
        if (sphere.GetType() != RigidBodyType::Sphere)
            return;

        glm::vec3 const center = ToGlm(sphere.GetPosition());
        float const radius = sphere.GetRadius();
        glm::ivec3 minCell;
        glm::ivec3 maxCell;
        SphereCellBounds(fluid, center, radius, minCell, maxCell);

        for (int i = minCell.x; i <= maxCell.x; ++i) {
            for (int j = minCell.y; j <= maxCell.y; ++j) {
                for (int k = minCell.z; k <= maxCell.z; ++k) {
                    if (!InsideSphere(CellCenter(fluid, i, j, k), center, radius))
                        continue;

                    fluid.m_s[fluid.index2GridOffset(glm::ivec3(i, j, k))] = 0.0f;
                }
            }
        }

        fluid.rebuildCellTypes();
    }

    void RigidFluidCoupler::ApplyPressureForces(Simulator const& fluid, RigidBodyItem& sphere, float pressureScale) const {
        if (sphere.GetType() != RigidBodyType::Sphere)
            return;

        glm::vec3 const center = ToGlm(sphere.GetPosition());
        float const radius = sphere.GetRadius();
        glm::ivec3 minCell;
        glm::ivec3 maxCell;
        SphereCellBounds(fluid, center, radius, minCell, maxCell);

        // ---- dynamic pressure forces ----

        glm::ivec3 const offsets[6] = {
            { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 },
            { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 }
        };
        float const faceArea = fluid.m_h * fluid.m_h;

        for (int i = minCell.x; i <= maxCell.x; ++i) {
            for (int j = minCell.y; j <= maxCell.y; ++j) {
                for (int k = minCell.z; k <= maxCell.z; ++k) {
                    glm::vec3 const c = CellCenter(fluid, i, j, k);
                    if (!InsideSphere(c, center, radius))
                        continue;

                    glm::vec3 direction(0.0f);
                    float pressureSum = 0.0f;
                    int fluidNeighbors = 0;

                    for (glm::ivec3 const& offset : offsets) {
                        glm::ivec3 n = glm::ivec3(i, j, k) + offset;
                        if (n.x < 0 || n.x >= fluid.m_iCellX ||
                            n.y < 0 || n.y >= fluid.m_iCellY ||
                            n.z < 0 || n.z >= fluid.m_iCellZ)
                            continue;

                        int nIdx = fluid.index2GridOffset(n);
                        if (fluid.m_type[nIdx] != FLUID_CELL)
                            continue;

                        direction += glm::normalize(c - CellCenter(fluid, n.x, n.y, n.z));
                        pressureSum += fluid.m_p[nIdx];
                        ++fluidNeighbors;
                    }

                    if (fluidNeighbors == 0 || glm::dot(direction, direction) < 1e-8f)
                        continue;

                    glm::vec3 force = glm::normalize(direction) *
                        (pressureSum / static_cast<float>(fluidNeighbors)) *
                        faceArea * pressureScale;
                    sphere.ApplyForceAtPoint(ToEigen(force), ToEigen(c));
                }
            }
        }

        // ---- buoyancy force (hydrostatic pressure integral) ----

        if (fluid.m_particleRestDensity <= 0.0f)
            return;

        // Compute far-field water surface height by averaging over all columns
        float sumSurfaceY = 0.0f;
        int numColumns = 0;
        for (int i = 0; i < fluid.m_iCellX; ++i) {
            for (int k = 0; k < fluid.m_iCellZ; ++k) {
                for (int j = fluid.m_iCellY - 1; j >= 0; --j) {
                    int idx = fluid.index2GridOffset(glm::ivec3(i, j, k));
                    if (fluid.m_type[idx] == FLUID_CELL) {
                        sumSurfaceY += -0.5f + (j + 0.5f) * fluid.m_h;
                        ++numColumns;
                        break;
                    }
                }
            }
        }

        if (numColumns == 0)
            return;

        float const avgSurfaceY = sumSurfaceY / static_cast<float>(numColumns);
        float const sphereBottom  = center.y - radius;
        float const submergedDepth = std::max(0.0f, std::min(2.0f * radius, avgSurfaceY - sphereBottom));

        if (submergedDepth <= 0.0f)
            return;

        float const h = submergedDepth;
        float const sphereVolume = (4.0f / 3.0f) * float(M_PI) * radius * radius * radius;
        float const capVolume = float(M_PI) * h * h * (3.0f * radius - h) / 3.0f;
        float const submergedFraction = capVolume / sphereVolume;
        float const g = glm::length(fluid.gravity);

        float const buoyancyScale    = 0.5f;
        float const buoyancyMagnitude = buoyancyScale * fluid.m_particleRestDensity * sphereVolume * submergedFraction * g;
        sphere.ApplyForce(ToEigen(buoyancyMagnitude * glm::normalize(-fluid.gravity)));
    }

} // namespace VCX::Labs::RBxFLIP
