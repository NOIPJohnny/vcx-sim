#include "FluidSimulator.h"
#include <cmath>
#include <algorithm>
#include <array>

namespace VCX::Labs::Fluid {

    namespace {
        inline int clampi(int x, int low, int hi) {return std::max(low, std::min(x, hi));}

        inline glm::vec3 staggeredOffset(int dir, float h) {
            glm::vec3 offset(0.5f * h);
            offset[dir] = 0.0f;
            return offset;
        }

        struct Trilerp {
            std::array<glm::ivec3, 8> nodes;
            std::array<float, 8>      weights;
            int                       count = 0;
        };

        inline Trilerp buildStencil(glm::vec3 const & pos,
                                    glm::vec3 const & gridMin,
                                    float             invSpacing,
                                    float             h,
                                    int               dir,
                                    int               cellX,
                                    int               cellY,
                                    int               cellZ) {
            Trilerp stencil;
            glm::vec3 relpos = (pos - gridMin - staggeredOffset(dir, h)) * invSpacing;
            int baseX = static_cast<int>(std::floor(relpos.x)),
                baseY = static_cast<int>(std::floor(relpos.y)),
                baseZ = static_cast<int>(std::floor(relpos.z));
            
            float fx = relpos.x - baseX,
                  fy = relpos.y - baseY,
                  fz = relpos.z - baseZ;
            
            for (int dx = 0; dx <= 1; ++dx) {
                float wx = dx ? fx : (1.0f - fx);
                int gx = baseX + dx;
                if (gx < 0 || gx >= cellX) continue;

                for (int dy = 0; dy <= 1; ++dy) {
                    float wy = dy ? fy : (1.0f - fy);
                    int gy = baseY + dy;
                    if (gy < 0 || gy >= cellY) continue;

                    for (int dz = 0; dz <= 1; ++dz) {
                        float wz = dz ? fz : (1.0f - fz);
                        int gz = baseZ + dz;
                        if (gz < 0 || gz >= cellZ) continue;

                        stencil.nodes[stencil.count] = glm::ivec3(gx, gy, gz);
                        stencil.weights[stencil.count] = wx * wy * wz;
                        stencil.count++;
                    }
                }
            }
            return stencil;
        }
    
        inline Trilerp buildCellCenterStencil(glm::vec3 const & pos,
                                        glm::vec3 const & gridMin,
                                        float             invSpacing,
                                        float             h,
                                        int               cellX,
                                        int               cellY,
                                        int               cellZ) {
            Trilerp stencil;
            glm::vec3 relpos = (pos - gridMin - glm::vec3(0.5f * h)) * invSpacing;

            int baseX = static_cast<int>(std::floor(relpos.x));
            int baseY = static_cast<int>(std::floor(relpos.y));
            int baseZ = static_cast<int>(std::floor(relpos.z));

            float fx = relpos.x - baseX;
            float fy = relpos.y - baseY;
            float fz = relpos.z - baseZ;

            for (int dx = 0; dx <= 1; ++dx) {
                float wx = dx ? fx : (1.0f - fx);
                int gx = baseX + dx;
                if (gx < 0 || gx >= cellX) continue;

                for (int dy = 0; dy <= 1; ++dy) {
                    float wy = dy ? fy : (1.0f - fy);
                    int gy = baseY + dy;
                    if (gy < 0 || gy >= cellY) continue;

                    for (int dz = 0; dz <= 1; ++dz) {
                        float wz = dz ? fz : (1.0f - fz);
                        int gz = baseZ + dz;
                        if (gz < 0 || gz >= cellZ) continue;

                        stencil.nodes[stencil.count] = glm::ivec3(gx, gy, gz);
                        stencil.weights[stencil.count] = wx * wy * wz;
                        stencil.count++;
                    }
                }
            }
            return stencil;
        }

        inline void resetGridData(std::vector<glm::vec3> & vel,
                                  std::vector<float> nearNum[3]) {
            std::fill(vel.begin(), vel.end(), glm::vec3(0.0f));
            for (int dir = 0; dir < 3; ++dir)
                std::fill(nearNum[dir].begin(), nearNum[dir].end(), 0.0f);
        }

        inline void rebuildCellTypes(std::vector<int> & type,
                                     std::vector<float> const & solidMask,
                                     std::vector<glm::vec3> const & particlePos,
                                     glm::vec3 const & gridMin,
                                     float invSpacing,
                                     int cellX,
                                     int cellY,
                                     int cellZ,
                                     int (*indexer)(glm::ivec3) = nullptr) = delete;

        template<typename Indexer>
        inline void rebuildCellTypes(std::vector<int> & type,
                                     std::vector<float> const & solidMask,
                                     std::vector<glm::vec3> const & particlePos,
                                     glm::vec3 const & gridMin,
                                     float invSpacing,
                                     int cellX,
                                     int cellY,
                                     int cellZ,
                                     Indexer indexer) {
            for (std::size_t idx = 0; idx < type.size(); ++idx)
                type[idx] = (solidMask[idx] == 0.0f) ? SOLID_CELL : EMPTY_CELL;

            for (glm::vec3 const & pos : particlePos) {
                glm::vec3 rel = (pos - gridMin) * invSpacing;

                int i = clampi(static_cast<int>(std::floor(rel.x)), 0, cellX - 1);
                int j = clampi(static_cast<int>(std::floor(rel.y)), 0, cellY - 1);
                int k = clampi(static_cast<int>(std::floor(rel.z)), 0, cellZ - 1);

                int idx = indexer(glm::ivec3(i, j, k));
                if (type[idx] != SOLID_CELL)
                    type[idx] = FLUID_CELL;
            }
        }

        inline void normalizeGridVelocities(std::vector<glm::vec3> & vel,
                                            std::vector<float> const nearNum[3],
                                            std::vector<int> const & type) {
            for (std::size_t idx = 0; idx < vel.size(); ++idx) {
                for (int dir = 0; dir < 3; ++dir) {
                    if (nearNum[dir][idx] > 1e-8f)
                        vel[idx][dir] /= nearNum[dir][idx];
                }

                if (type[idx] == SOLID_CELL)
                    vel[idx] = glm::vec3(0.0f);
            }
        }

        template<typename Indexer>
        inline void extrapolateVelocityField(std::vector<glm::vec3> & gridField,
                                             std::vector<float> const nearNum[3],
                                             std::vector<int> const & type,
                                             int cellX,
                                             int cellY,
                                             int cellZ,
                                             Indexer indexer) {
            constexpr int numLayers = 3;
            const glm::ivec3 neighborOffsets[6] = {
                {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
            };

            for (int dir = 0; dir < 3; ++dir) {
                std::vector<char> valid(type.size(), 0);
                for (std::size_t idx = 0; idx < type.size(); ++idx)
                    valid[idx] = (nearNum[dir][idx] > 1e-8f) ? 1 : 0;

                for (int iter = 0; iter < numLayers; ++iter) {
                    std::vector<char> nextValid = valid;
                    std::vector<glm::vec3> nextField = gridField;

                    for (int i = 0; i < cellX; ++i)
                        for (int j = 0; j < cellY; ++j)
                            for (int k = 0; k < cellZ; ++k) {
                                int idx = indexer(glm::ivec3(i, j, k));

                                if (valid[idx] || type[idx] == SOLID_CELL) continue;

                                float sum = 0.0f;
                                int count = 0;

                                for (auto const& offset : neighborOffsets) {
                                    glm::ivec3 n = glm::ivec3(i, j, k) + offset;
                                    if (n.x >= 0 && n.x < cellX &&
                                        n.y >= 0 && n.y < cellY &&
                                        n.z >= 0 && n.z < cellZ) {
                                        int nIdx = indexer(n);
                                        if (valid[nIdx]) {
                                            sum += gridField[nIdx][dir];
                                            count++;
                                        }
                                    }
                                }

                                if (count > 0) {
                                    nextField[idx][dir] = sum / static_cast<float>(count);
                                    nextValid[idx] = 1;
                                }
                            }

                    valid = std::move(nextValid);
                    gridField = std::move(nextField);
                }
            }
        }
    } // anonymous namespace for helper functions

    void Simulator::integrateParticles(float timeStep) {
        if (m_iNumSpheres == 0 || timeStep <= 0.0f)
            return;

        for (int i = 0; i < m_iNumSpheres; ++i) {
            m_particleVel[i] += gravity * timeStep;
            m_particlePos[i] += m_particleVel[i] * timeStep;
        }
    }

    void Simulator::handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRadius, glm::vec3 obstacleVel) {
        const float r = m_particleRadius;
        const float tankMin = -0.5f + m_h + r;
        const float tankMax = 0.5f - m_h - r;
        
        for (int i = 0; i < m_iNumSpheres; ++i) {
            glm::vec3& pos = m_particlePos[i];
            glm::vec3& v = m_particleVel[i];

            // grid
            for (int axis = 0; axis < 3; ++axis) {
                if (pos[axis] < tankMin) {
                    pos[axis] = tankMin;
                    if (v[axis] < 0.0f) v[axis] = 0.0f;
                }
                if (pos[axis] > tankMax) {
                    pos[axis] = tankMax;
                    if (v[axis] > 0.0f) v[axis] = 0.0f;
                }
            }

            //obstacle
            if (obstacleRadius <= 0.0f) continue;

            glm::vec3 dir = pos - obstaclePos; // pointing out
            float dist2 = glm::dot(dir, dir);
            float minDist = obstacleRadius + r;

            if (dist2 > minDist * minDist) continue;

            const float dist = std::sqrt(dist2);
            glm::vec3 n = dist > 1e-6f ? glm::normalize(dir) : glm::vec3(0.0f, 1.0f, 0.0f);
            pos = obstaclePos + n * minDist; // push

            float v_n = glm::dot(v - obstacleVel, n);
            if (v_n < 0.0f) v -= v_n * n;
        }
    }

    int Simulator::index2GridOffset(glm::ivec3 index) {
        return index.x * (m_iCellY * m_iCellZ) + index.y * m_iCellZ + index.z;
    }

    bool Simulator::isValidVelocity(int i, int j, int k, int dir) {
        if (i < 0 || j < 0 || k < 0 || i >= m_iCellX || j >= m_iCellY || k >= m_iCellZ)
            return false;
        glm::ivec3 a(i, j, k);
        glm::ivec3 b = a;
        b[dir] -= 1;
        if (b.x < 0 || b.y < 0 || b.z < 0 || b.x >= m_iCellX || b.y >= m_iCellY || b.z >= m_iCellZ)
            return false;
        return m_s[index2GridOffset(a)] > 0.0f || m_s[index2GridOffset(b)] > 0.0f;
    }

    void Simulator::transferVelocities(bool toGrid, float flipRatio) {
        const glm::vec3 gridMin(-0.5f, -0.5f, -0.5f);

        auto accumulateParticleToGrid = [&](glm::vec3 const & pos, glm::vec3 const & vel) {
            for (int dir = 0; dir < 3; ++dir) {
                Trilerp stencil = buildStencil(
                    pos, gridMin, m_fInvSpacing, m_h, dir, m_iCellX, m_iCellY, m_iCellZ);

                for (int s = 0; s < stencil.count; ++s) {
                    glm::ivec3 const node = stencil.nodes[s];
                    float const w = stencil.weights[s];
                    if (!isValidVelocity(node.x, node.y, node.z, dir)) continue;
                    int idx = index2GridOffset(node);
                    m_vel[idx][dir] += w * vel[dir];
                    m_near_num[dir][idx] += w;
                }
            }
        };

        auto sampleGridVelocity = [&](glm::vec3 const & pos,
                                  std::vector<glm::vec3> const & gridField) -> glm::vec3 {
            glm::vec3 sampled(0.0f);

            for (int dir = 0; dir < 3; ++dir) {
                Trilerp stencil = buildStencil(
                    pos, gridMin, m_fInvSpacing, m_h, dir, m_iCellX, m_iCellY, m_iCellZ);

                float sum = 0.0f;
                float sumW = 0.0f;

                for (int s = 0; s < stencil.count; ++s) {
                    glm::ivec3 const node = stencil.nodes[s];
                    float const w = stencil.weights[s];

                    if (!isValidVelocity(node.x, node.y, node.z, dir)) continue;

                    int idx = index2GridOffset(node);
                    sum += w * gridField[idx][dir];
                    sumW += w;
                }

                if (sumW > 1e-8f)
                    sampled[dir] = sum / sumW;
            }

            return sampled;
        };

        if (toGrid) {
            resetGridData(m_vel, m_near_num);
            rebuildCellTypes(
                m_type, m_s, m_particlePos, gridMin, m_fInvSpacing,
                m_iCellX, m_iCellY, m_iCellZ,
                [&](glm::ivec3 index) { return index2GridOffset(index); });

            for (int p = 0; p < static_cast<int>(m_particlePos.size()); ++p)
                accumulateParticleToGrid(m_particlePos[p], m_particleVel[p]);
            
            normalizeGridVelocities(m_vel, m_near_num, m_type);
            m_pre_vel = m_vel;
            return;
        }

        extrapolateVelocityField(
            m_vel, m_near_num, m_type, m_iCellX, m_iCellY, m_iCellZ,
            [&](glm::ivec3 index) { return index2GridOffset(index); });
        extrapolateVelocityField(
            m_pre_vel, m_near_num, m_type, m_iCellX, m_iCellY, m_iCellZ,
            [&](glm::ivec3 index) { return index2GridOffset(index); });

        for (int p = 0; p < m_particlePos.size(); ++p) {
            const glm::vec3 oldVel = m_particleVel[p];

            glm::vec3 picVel = sampleGridVelocity(m_particlePos[p], m_vel);
            glm::vec3 oldGridVel = sampleGridVelocity(m_particlePos[p], m_pre_vel);
            glm::vec3 flipVel = oldVel + (picVel - oldGridVel);

            m_particleVel[p] = (1.0f - flipRatio) * picVel + flipRatio * flipVel;
        }
    }

    void Simulator::transferVelocitiesAPIC(bool toGrid) {
        const glm::vec3 gridMin(-0.5f, -0.5f, -0.5f);

        auto gridSamplePosition = [&](glm::ivec3 const & node, int dir) -> glm::vec3 {
            glm::vec3 pos = gridMin + glm::vec3(node) * m_h + staggeredOffset(dir, m_h);
            return pos;
        };

        if (toGrid) {
            resetGridData(m_vel, m_near_num);
            rebuildCellTypes(
                m_type, m_s, m_particlePos, gridMin, m_fInvSpacing,
                m_iCellX, m_iCellY, m_iCellZ,
                [&](glm::ivec3 index) { return index2GridOffset(index); });

            for (int p = 0; p < static_cast<int>(m_particlePos.size()); ++p) {
                glm::vec3 const & pos = m_particlePos[p];
                glm::vec3 const & vel = m_particleVel[p];
                glm::mat3 const & C = m_particleC[p];

                for (int dir = 0; dir < 3; ++dir) {
                    Trilerp stencil = buildStencil(
                        pos, gridMin, m_fInvSpacing, m_h, dir, m_iCellX, m_iCellY, m_iCellZ);

                    for (int s = 0; s < stencil.count; ++s) {
                        glm::ivec3 const node = stencil.nodes[s];
                        float const w = stencil.weights[s];
                        if (!isValidVelocity(node.x, node.y, node.z, dir)) continue;

                        glm::vec3 dx = gridSamplePosition(node, dir) - pos;
                        float contributed = vel[dir] + glm::dot(C * dx, glm::vec3(dir == 0, dir == 1, dir == 2));

                        int idx = index2GridOffset(node);
                        m_vel[idx][dir] += w * contributed;
                        m_near_num[dir][idx] += w;
                    }
                }
            }

            normalizeGridVelocities(m_vel, m_near_num, m_type);
            m_pre_vel = m_vel;
            return;
        }

        extrapolateVelocityField(
            m_vel, m_near_num, m_type, m_iCellX, m_iCellY, m_iCellZ,
            [&](glm::ivec3 index) { return index2GridOffset(index); });
        extrapolateVelocityField(
            m_pre_vel, m_near_num, m_type, m_iCellX, m_iCellY, m_iCellZ,
            [&](glm::ivec3 index) { return index2GridOffset(index); });

        for (int p = 0; p < static_cast<int>(m_particlePos.size()); ++p) {
            glm::vec3 const & pos = m_particlePos[p];

            glm::vec3 newVel(0.0f);
            glm::vec3 rowX(0.0f);
            glm::vec3 rowY(0.0f);
            glm::vec3 rowZ(0.0f);

            for (int dir = 0; dir < 3; ++dir) {
                Trilerp stencil = buildStencil(
                    pos, gridMin, m_fInvSpacing, m_h, dir, m_iCellX, m_iCellY, m_iCellZ);

                float sum = 0.0f;
                float sumW = 0.0f;

                for (int s = 0; s < stencil.count; ++s) {
                    glm::ivec3 const node = stencil.nodes[s];
                    float const w = stencil.weights[s];
                    if (!isValidVelocity(node.x, node.y, node.z, dir)) continue;

                    int idx = index2GridOffset(node);
                    float u = m_vel[idx][dir];
                    sum += w * u;
                    sumW += w;
                }

                if (sumW > 1e-8f)
                    newVel[dir] = sum / sumW;

                // Recover only the local affine variation, not the constant part.
                // This is noticeably more stable near free surfaces / boundaries.
                glm::vec3 row(0.0f);
                for (int s = 0; s < stencil.count; ++s) {
                    glm::ivec3 const node = stencil.nodes[s];
                    float const w = stencil.weights[s];
                    if (!isValidVelocity(node.x, node.y, node.z, dir)) continue;

                    int idx = index2GridOffset(node);
                    glm::vec3 dx = gridSamplePosition(node, dir) - pos;
                    float du = m_vel[idx][dir] - newVel[dir];
                    row += w * du * dx;
                }

                if (dir == 0) rowX = row;
                if (dir == 1) rowY = row;
                if (dir == 2) rowZ = row;
            }

            m_particleVel[p] = newVel;

            float const scale = 4.0f / (m_h * m_h);
            glm::mat3 C(0.0f);
            C[0] = glm::vec3(rowX.x, rowY.x, rowZ.x);
            C[1] = glm::vec3(rowX.y, rowY.y, rowZ.y);
            C[2] = glm::vec3(rowX.z, rowY.z, rowZ.z);

            // A little damping helps this approximate staggered-grid APIC stay stable.
            m_particleC[p] = 0.95f * (C * scale);
        }
    }

    void Simulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift) {
        if (dt <= 0.0f) return;

        std::fill(m_p.begin(), m_p.end(), 0.0f);

        const float driftK = 1.0f;
        for (int iter = 0; iter < numIters; ++iter)
            for (int i = 1; i < m_iCellX - 1; ++i)
                for (int j = 1; j < m_iCellY - 1; ++j)
                    for (int k = 1; k < m_iCellZ - 1; ++k) {
                        int center = index2GridOffset(glm::ivec3(i, j, k));

                        if (m_type[center] != FLUID_CELL) continue;

                        int left  = index2GridOffset(glm::ivec3(i - 1, j, k));
                        int right = index2GridOffset(glm::ivec3(i + 1, j, k));
                        int down  = index2GridOffset(glm::ivec3(i, j - 1, k));
                        int up    = index2GridOffset(glm::ivec3(i, j + 1, k));
                        int back  = index2GridOffset(glm::ivec3(i, j, k - 1));
                        int front = index2GridOffset(glm::ivec3(i, j, k + 1));

                        float sx0 = m_s[left];
                        float sx1 = m_s[right];
                        float sy0 = m_s[down];
                        float sy1 = m_s[up];
                        float sz0 = m_s[back];
                        float sz1 = m_s[front];

                        float s = sx0 + sx1 + sy0 + sy1 + sz0 + sz1;
                        if (s <= 0.0f) continue;

                        float div =
                            m_vel[right].x - m_vel[center].x +
                            m_vel[up].y    - m_vel[center].y +
                            m_vel[front].z - m_vel[center].z;

                        if (compensateDrift && m_particleRestDensity > 0.0f) {
                            float compression = m_particleDensity[center] - m_particleRestDensity;
                            if (compression > 0.0f) div -= driftK * compression;
                        }

                        float p = -div / s;
                        p *= overRelaxation;

                        m_p[center] += p;
                        m_vel[center].x -= sx0 * p;
                        m_vel[right].x  += sx1 * p;
                        m_vel[center].y -= sy0 * p;
                        m_vel[up].y     += sy1 * p;
                        m_vel[center].z -= sz0 * p;
                        m_vel[front].z  += sz1 * p;
                    }
    }

    void Simulator::solveIncompressibilityCG(float dt, bool compensateDrift, bool usePreconditioner) {
        if (dt <= 0.0f) return;

        std::fill(m_p.begin(), m_p.end(), 0.0f);

        const float driftK = 1.0f;

        // build mapping
        std::vector<int> cellToUnknown(m_iNumCells, -1);
        std::vector<int> unknownToCell;
        unknownToCell.reserve(m_iNumCells);

        for (int idx = 0; idx < m_iNumCells; ++idx)
            if (m_type[idx] == FLUID_CELL) {
                cellToUnknown[idx] = static_cast<int>(unknownToCell.size());
                unknownToCell.push_back(idx);
            }

        const int n = static_cast<int>(unknownToCell.size());
        if (n == 0) return;

        Eigen::SparseMatrix<float> A(n, n);
        Eigen::VectorXf b(n);
        Eigen::VectorXf x(n);
        x.setZero();

        std::vector<Eigen::Triplet<float>> triplets;
        triplets.reserve(n * 7);

        auto addNeighborContribution = [&](int row,
                                        int neighborCell,
                                        float sNeighbor,
                                        float & diag) {
            if (sNeighbor <= 0.0f) return;
            diag += sNeighbor;
            int col = cellToUnknown[neighborCell];
            if (col >= 0)
                triplets.emplace_back(row, col, -sNeighbor);
        };

        for (int row = 0; row < n; ++row) {
            int center = unknownToCell[row];

            int i = center / (m_iCellY * m_iCellZ);
            int rem = center % (m_iCellY * m_iCellZ);
            int j = rem / m_iCellZ;
            int k = rem % m_iCellZ;

            int left  = index2GridOffset(glm::ivec3(i - 1, j, k));
            int right = index2GridOffset(glm::ivec3(i + 1, j, k));
            int down  = index2GridOffset(glm::ivec3(i, j - 1, k));
            int up    = index2GridOffset(glm::ivec3(i, j + 1, k));
            int back  = index2GridOffset(glm::ivec3(i, j, k - 1));
            int front = index2GridOffset(glm::ivec3(i, j, k + 1));

            float sx0 = m_s[left];
            float sx1 = m_s[right];
            float sy0 = m_s[down];
            float sy1 = m_s[up];
            float sz0 = m_s[back];
            float sz1 = m_s[front];

            float diag = 0.0f;
            addNeighborContribution(row, left,  sx0, diag);
            addNeighborContribution(row, right, sx1, diag);
            addNeighborContribution(row, down,  sy0, diag);
            addNeighborContribution(row, up,    sy1, diag);
            addNeighborContribution(row, back,  sz0, diag);
            addNeighborContribution(row, front, sz1, diag);

            triplets.emplace_back(row, row, diag);

            float div =
                m_vel[right].x - m_vel[center].x +
                m_vel[up].y    - m_vel[center].y +
                m_vel[front].z - m_vel[center].z;

            if (compensateDrift && m_particleRestDensity > 0.0f) {
                float compression = m_particleDensity[center] - m_particleRestDensity;
                if (compression > 0.0f)
                    div -= driftK * compression;
            }

            b[row] = -div;
        }

        A.setFromTriplets(triplets.begin(), triplets.end());

        if (usePreconditioner) {
            Eigen::ConjugateGradient<
                Eigen::SparseMatrix<float>,
                Eigen::Lower | Eigen::Upper,
                Eigen::IncompleteCholesky<float>
            > solver;

            solver.setMaxIterations(std::max(20, n));
            solver.setTolerance(1e-4f);
            solver.compute(A);
            if (solver.info() == Eigen::Success)
                x = solver.solve(b);
        }
        else {
            Eigen::ConjugateGradient<Eigen::SparseMatrix<float>, Eigen::Lower | Eigen::Upper> solver;

            solver.setMaxIterations(std::max(20, n));
            solver.setTolerance(1e-4f);
            solver.compute(A);
            if (solver.info() == Eigen::Success) {
                x = solver.solve(b);
            }
        }

        for (int row = 0; row < n; ++row)
            m_p[unknownToCell[row]] = x[row];

        for (int row = 0; row < n; ++row) {
            int center = unknownToCell[row];

            int i = center / (m_iCellY * m_iCellZ);
            int rem = center % (m_iCellY * m_iCellZ);
            int j = rem / m_iCellZ;
            int k = rem % m_iCellZ;

            int right = index2GridOffset(glm::ivec3(i + 1, j, k));
            int up    = index2GridOffset(glm::ivec3(i, j + 1, k));
            int front = index2GridOffset(glm::ivec3(i, j, k + 1));

            if (m_s[right] > 0.0f)
                m_vel[right].x -= m_p[right] - m_p[center];
            if (m_s[up] > 0.0f)
                m_vel[up].y -= m_p[up] - m_p[center];
            if (m_s[front] > 0.0f)
                m_vel[front].z -= m_p[front] - m_p[center];
        }

        for (int row = 0; row < n; ++row) {
            int center = unknownToCell[row];

            int i = center / (m_iCellY * m_iCellZ);
            int rem = center % (m_iCellY * m_iCellZ);
            int j = rem / m_iCellZ;
            int k = rem % m_iCellZ;

            int left  = index2GridOffset(glm::ivec3(i - 1, j, k));
            int down  = index2GridOffset(glm::ivec3(i, j - 1, k));
            int back  = index2GridOffset(glm::ivec3(i, j, k - 1));

            if (m_s[left] > 0.0f && cellToUnknown[left] < 0)
                m_vel[center].x -= m_p[center];   // p_left = 0
            if (m_s[down] > 0.0f && cellToUnknown[down] < 0)
                m_vel[center].y -= m_p[center];
            if (m_s[back] > 0.0f && cellToUnknown[back] < 0)
                m_vel[center].z -= m_p[center];
        }
    }


    void Simulator::updateParticleDensity() {
        const glm::vec3 gridMin(-0.5f, -0.5f, -0.5f);

        std::fill(m_particleDensity.begin(), m_particleDensity.end(), 0.0f);

        for (glm::vec3 const & pos : m_particlePos) {
            Trilerp stencil = buildCellCenterStencil(
                pos, gridMin, m_fInvSpacing, m_h, m_iCellX, m_iCellY, m_iCellZ);

            for (int s = 0; s < stencil.count; ++s) {
                int idx = index2GridOffset(stencil.nodes[s]);
                m_particleDensity[idx] += stencil.weights[s];
            }
        }

        if (m_particleRestDensity == 0.0f) {
            float sum = 0.0f;
            int count = 0;

            for (int idx = 0; idx < m_iNumCells; ++idx) {
                if (m_type[idx] == FLUID_CELL) {
                    sum += m_particleDensity[idx];
                    count++;
                }
            }

            if (count > 0)
                m_particleRestDensity = sum / static_cast<float>(count);
        }
    }

    void Simulator::pushParticlesApart(int numIters) {
        if (numIters <= 0 || m_particlePos.empty()) return;

        const glm::vec3 gridMin(-0.5f, -0.5f, -0.5f);
        const float minDist = 2.0f * m_particleRadius;
        const float minDist2 = minDist * minDist;

        auto particleCell = [&](glm::vec3 const & pos) -> glm::ivec3 {
            glm::vec3 rel = (pos - gridMin) * m_fInvSpacing;

            int i = clampi(static_cast<int>(std::floor(rel.x)), 0, m_iCellX - 1);
            int j = clampi(static_cast<int>(std::floor(rel.y)), 0, m_iCellY - 1);
            int k = clampi(static_cast<int>(std::floor(rel.z)), 0, m_iCellZ - 1);
            return glm::ivec3(i, j, k);
        };

        for (int iter = 0; iter < numIters; ++iter) {
            std::fill(m_hashtableindex.begin(), m_hashtableindex.end(), 0);

            for (int p = 0; p < static_cast<int>(m_particlePos.size()); ++p) {
                glm::ivec3 cell = particleCell(m_particlePos[p]);
                int idx = index2GridOffset(cell);
                m_hashtableindex[idx + 1]++;
            }

            for (int c = 0; c < m_iNumCells; ++c)
                m_hashtableindex[c + 1] += m_hashtableindex[c]; // prefix

            // Fill particle ids into the contiguous bucket array
            std::vector<int> start = m_hashtableindex;
            for (int p = 0; p < static_cast<int>(m_particlePos.size()); ++p) {
                glm::ivec3 cell = particleCell(m_particlePos[p]);
                int idx = index2GridOffset(cell);
                m_hashtable[start[idx]++] = p;
            }

            // Gauss-Seidel style particle separation
            for (int p = 0; p < static_cast<int>(m_particlePos.size()); ++p) {
                glm::ivec3 cell = particleCell(m_particlePos[p]);

                for (int dx = -1; dx <= 1; ++dx) {
                    int nx = cell.x + dx;
                    if (nx < 0 || nx >= m_iCellX) continue;

                    for (int dy = -1; dy <= 1; ++dy) {
                        int ny = cell.y + dy;
                        if (ny < 0 || ny >= m_iCellY) continue;

                        for (int dz = -1; dz <= 1; ++dz) {
                            int nz = cell.z + dz;
                            if (nz < 0 || nz >= m_iCellZ) continue;

                            int ncell = index2GridOffset(glm::ivec3(nx, ny, nz));
                            int begin = m_hashtableindex[ncell];
                            int end   = m_hashtableindex[ncell + 1];

                            for (int t = begin; t < end; ++t) {
                                int q = m_hashtable[t];
                                if (q <= p) continue;

                                glm::vec3 dir = m_particlePos[p] - m_particlePos[q];
                                float dist2 = glm::dot(dir, dir);
                                if (dist2 >= minDist2) continue;

                                float dist = std::sqrt(dist2);
                                glm::vec3 n;

                                if (dist > 1e-6f) n = dir / dist;
                                else {
                                    n = glm::vec3(1.0f, 0.0f, 0.0f);
                                    dist = 0.0f;
                                }

                                float delta = 0.5f * (minDist - dist);
                                glm::vec3 offset = delta * n;

                                m_particlePos[p] += offset;
                                m_particlePos[q] -= offset;
                            }
                        }
                    }
                }
            }
        }
    }

    void Simulator::updateParticleColors() {
        glm::vec3 const lowColor(0.15f, 0.45f, 1.0f);
        glm::vec3 const highColor(1.0f, 0.55f, 0.15f);

        for (int i = 0; i < m_iNumSpheres; ++i) {
            float const speed = glm::length(m_particleVel[i]);
            float const t = std::clamp(speed * 0.15f, 0.0f, 1.0f);
            m_particleColor[i] = glm::mix(lowColor, highColor, t);
        }
    }

} // namespace VCX::Labs::Fluid
