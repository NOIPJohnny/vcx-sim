#include "FluidSurface.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace VCX::Labs::Coupling {

    namespace {
        constexpr glm::vec3 kGridMin(-0.5f);

        int ClampInt(int value, int lo, int hi) {
            return std::max(lo, std::min(value, hi));
        }

        glm::vec3 GridPoint(Simulator const & sim, int i, int j, int k) {
            return kGridMin + sim.m_h * glm::vec3(i, j, k);
        }

        class ParticleField {
        public:
            ParticleField(Simulator const & sim, float radiusScale) :
                _sim(sim),
                _radius(std::max(sim.m_particleRadius * radiusScale, sim.m_h * 0.45f)),
                _searchRange(std::max(1, static_cast<int>(std::ceil(_radius * sim.m_fInvSpacing)) + 1)),
                _buckets(static_cast<std::size_t>(sim.m_iNumCells)) {
                for (int p = 0; p < static_cast<int>(_sim.m_particlePos.size()); ++p)
                    _buckets[CellIndex(ParticleCell(_sim.m_particlePos[p]))].push_back(p);
            }

            float Sample(glm::vec3 const & position) const {
                glm::ivec3 const cell = ParticleCell(position);
                float best = std::numeric_limits<float>::max();

                // Only nearby buckets can affect the union-of-particles signed distance.
                for (int dx = -_searchRange; dx <= _searchRange; ++dx) {
                    int const x = cell.x + dx;
                    if (x < 0 || x >= _sim.m_iCellX) continue;

                    for (int dy = -_searchRange; dy <= _searchRange; ++dy) {
                        int const y = cell.y + dy;
                        if (y < 0 || y >= _sim.m_iCellY) continue;

                        for (int dz = -_searchRange; dz <= _searchRange; ++dz) {
                            int const z = cell.z + dz;
                            if (z < 0 || z >= _sim.m_iCellZ) continue;

                            for (int p : _buckets[CellIndex(glm::ivec3(x, y, z))]) {
                                float const distance = glm::length(position - _sim.m_particlePos[p]) - _radius;
                                best = std::min(best, distance);
                            }
                        }
                    }
                }

                return best == std::numeric_limits<float>::max() ? _radius : best;
            }

            glm::vec3 Normal(glm::vec3 const & position) const {
                glm::ivec3 const cell = ParticleCell(position);
                float bestDist2 = std::numeric_limits<float>::max();
                glm::vec3 bestDir(0.0f, 1.0f, 0.0f);

                for (int dx = -_searchRange; dx <= _searchRange; ++dx) {
                    int const x = cell.x + dx;
                    if (x < 0 || x >= _sim.m_iCellX) continue;

                    for (int dy = -_searchRange; dy <= _searchRange; ++dy) {
                        int const y = cell.y + dy;
                        if (y < 0 || y >= _sim.m_iCellY) continue;

                        for (int dz = -_searchRange; dz <= _searchRange; ++dz) {
                            int const z = cell.z + dz;
                            if (z < 0 || z >= _sim.m_iCellZ) continue;

                            for (int p : _buckets[CellIndex(glm::ivec3(x, y, z))]) {
                                glm::vec3 const dir = position - _sim.m_particlePos[p];
                                float const dist2 = glm::dot(dir, dir);
                                if (dist2 < bestDist2 && dist2 > 1e-12f) {
                                    bestDist2 = dist2;
                                    bestDir = dir;
                                }
                            }
                        }
                    }
                }

                float const len = glm::length(bestDir);
                return len > 1e-6f ? bestDir / len : glm::vec3(0.0f, 1.0f, 0.0f);
            }

        private:
            glm::ivec3 ParticleCell(glm::vec3 const & position) const {
                glm::vec3 const rel = (position - kGridMin) * _sim.m_fInvSpacing;
                return glm::ivec3(
                    ClampInt(static_cast<int>(std::floor(rel.x)), 0, _sim.m_iCellX - 1),
                    ClampInt(static_cast<int>(std::floor(rel.y)), 0, _sim.m_iCellY - 1),
                    ClampInt(static_cast<int>(std::floor(rel.z)), 0, _sim.m_iCellZ - 1)
                );
            }

            int CellIndex(glm::ivec3 const & cell) const {
                return (cell.x * _sim.m_iCellY + cell.y) * _sim.m_iCellZ + cell.z;
            }

            Simulator const &              _sim;
            float                          _radius;
            int                            _searchRange;
            std::vector<std::vector<int>>  _buckets;
        };

        glm::vec3 Interpolate(glm::vec3 const & a, glm::vec3 const & b, float fa, float fb) {
            float const denom = fa - fb;
            float const t = std::abs(denom) > 1e-8f ? std::clamp(fa / denom, 0.0f, 1.0f) : 0.5f;
            return a + t * (b - a);
        }

        void AddTriangle(
            FluidSurfaceMesh & mesh,
            ParticleField const & field,
            glm::vec3 a,
            glm::vec3 b,
            glm::vec3 c,
            glm::vec3 const & desiredNormal) {
            glm::vec3 const faceNormal = glm::cross(b - a, c - a);
            if (glm::dot(faceNormal, faceNormal) < 1e-14f)
                return;

            if (glm::dot(faceNormal, desiredNormal) < 0.0f)
                std::swap(b, c);

            std::uint32_t const base = static_cast<std::uint32_t>(mesh.positions.size());
            mesh.positions.push_back(a);
            mesh.positions.push_back(b);
            mesh.positions.push_back(c);
            mesh.normals.push_back(field.Normal(a));
            mesh.normals.push_back(field.Normal(b));
            mesh.normals.push_back(field.Normal(c));
            mesh.indices.push_back(base);
            mesh.indices.push_back(base + 1);
            mesh.indices.push_back(base + 2);
        }

        void PolygonizeTet(
            FluidSurfaceMesh & mesh,
            ParticleField const & field,
            std::array<glm::vec3, 4> const & p,
            std::array<float, 4> const & f) {
            std::array<int, 4> inside {};
            std::array<int, 4> outside {};
            int insideCount = 0;
            int outsideCount = 0;

            for (int i = 0; i < 4; ++i) {
                if (f[i] < 0.0f)
                    inside[insideCount++] = i;
                else
                    outside[outsideCount++] = i;
            }

            if (insideCount == 0 || insideCount == 4)
                return;

            glm::vec3 insideCenter(0.0f);
            glm::vec3 outsideCenter(0.0f);
            for (int i = 0; i < insideCount; ++i) insideCenter += p[inside[i]];
            for (int i = 0; i < outsideCount; ++i) outsideCenter += p[outside[i]];
            insideCenter /= static_cast<float>(insideCount);
            outsideCenter /= static_cast<float>(outsideCount);
            glm::vec3 const desiredNormal = outsideCenter - insideCenter;

            if (insideCount == 1 || insideCount == 3) {
                int const single = insideCount == 1 ? inside[0] : outside[0];
                std::array<int, 3> const others = insideCount == 1
                    ? std::array<int, 3>{ outside[0], outside[1], outside[2] }
                    : std::array<int, 3>{ inside[0], inside[1], inside[2] };

                std::array<glm::vec3, 3> tri {
                    Interpolate(p[single], p[others[0]], f[single], f[others[0]]),
                    Interpolate(p[single], p[others[1]], f[single], f[others[1]]),
                    Interpolate(p[single], p[others[2]], f[single], f[others[2]])
                };
                AddTriangle(mesh, field, tri[0], tri[1], tri[2], desiredNormal);
                return;
            }

            int const i0 = inside[0];
            int const i1 = inside[1];
            int const o0 = outside[0];
            int const o1 = outside[1];

            glm::vec3 const a = Interpolate(p[i0], p[o0], f[i0], f[o0]);
            glm::vec3 const b = Interpolate(p[i1], p[o0], f[i1], f[o0]);
            glm::vec3 const c = Interpolate(p[i1], p[o1], f[i1], f[o1]);
            glm::vec3 const d = Interpolate(p[i0], p[o1], f[i0], f[o1]);

            AddTriangle(mesh, field, a, b, c, desiredNormal);
            AddTriangle(mesh, field, a, c, d, desiredNormal);
        }
    } // namespace

    FluidSurfaceMesh BuildFluidSurface(Simulator const & sim, float radiusScale) {
        FluidSurfaceMesh mesh;
        if (sim.m_particlePos.empty() ||
            sim.m_iCellX <= 1 || sim.m_iCellY <= 1 || sim.m_iCellZ <= 1 ||
            sim.m_iNumCells <= 0 || sim.m_h <= 0.0f || sim.m_fInvSpacing <= 0.0f ||
            sim.m_particleRadius <= 0.0f) {
            return mesh;
        }

        ParticleField field(sim, radiusScale);
        std::vector<float> samples(static_cast<std::size_t>(sim.m_iNumCells), 0.0f);
        for (int i = 0; i < sim.m_iCellX; ++i)
            for (int j = 0; j < sim.m_iCellY; ++j)
                for (int k = 0; k < sim.m_iCellZ; ++k)
                    samples[(i * sim.m_iCellY + j) * sim.m_iCellZ + k] = field.Sample(GridPoint(sim, i, j, k));

        constexpr std::array<std::array<int, 4>, 6> tets {{
            {{ 0, 5, 1, 6 }},
            {{ 0, 1, 2, 6 }},
            {{ 0, 2, 3, 6 }},
            {{ 0, 3, 7, 6 }},
            {{ 0, 7, 4, 6 }},
            {{ 0, 4, 5, 6 }},
        }};

        constexpr std::array<glm::ivec3, 8> offsets {{
            { 0, 0, 0 },
            { 1, 0, 0 },
            { 1, 1, 0 },
            { 0, 1, 0 },
            { 0, 0, 1 },
            { 1, 0, 1 },
            { 1, 1, 1 },
            { 0, 1, 1 },
        }};

        for (int i = 0; i + 1 < sim.m_iCellX; ++i) {
            for (int j = 0; j + 1 < sim.m_iCellY; ++j) {
                for (int k = 0; k + 1 < sim.m_iCellZ; ++k) {
                    std::array<glm::vec3, 8> cubePos {};
                    std::array<float, 8> cubeField {};
                    bool hasInside = false;
                    bool hasOutside = false;

                    for (int c = 0; c < 8; ++c) {
                        glm::ivec3 const node = glm::ivec3(i, j, k) + offsets[c];
                        cubePos[c] = GridPoint(sim, node.x, node.y, node.z);
                        cubeField[c] = samples[(node.x * sim.m_iCellY + node.y) * sim.m_iCellZ + node.z];
                        hasInside = hasInside || cubeField[c] < 0.0f;
                        hasOutside = hasOutside || cubeField[c] >= 0.0f;
                    }

                    if (! hasInside || ! hasOutside)
                        continue;

                    for (auto const & tet : tets) {
                        std::array<glm::vec3, 4> tetPos {
                            cubePos[tet[0]], cubePos[tet[1]], cubePos[tet[2]], cubePos[tet[3]]
                        };
                        std::array<float, 4> tetField {
                            cubeField[tet[0]], cubeField[tet[1]], cubeField[tet[2]], cubeField[tet[3]]
                        };
                        PolygonizeTet(mesh, field, tetPos, tetField);
                    }
                }
            }
        }

        return mesh;
    }

} // namespace VCX::Labs::Coupling
