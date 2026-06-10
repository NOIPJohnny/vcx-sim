#include "FEMSoftBodyBuilder.h"
#include "FEMSystem.h"

#include <algorithm>
#include <array>
#include <map>
#include <vector>

#include <glm/geometric.hpp>

namespace VCX::Labs::FEM {

    int SoftBodyTypeCount() {
        return 3;
    }

    char const * SoftBodyTypeName(SoftBodyType type) {
        switch (type) {
        case SoftBodyType::GridBlock:
            return "Grid Block";
        case SoftBodyType::Sphere:
            return "Sphere";
        case SoftBodyType::TeddyBear:
            return "Teddy Bear";
        }
        return "Unknown";
    }

    static void ClearStructure(FEMSystem & system) {
        system.positions.clear();
        system.velocities.clear();
        system.masses.clear();
        system.fixed.clear();
        system.forces.clear();
        system.externalForces.clear();
        system.tets.clear();
        system.surfaceFaces.clear();
    }

    static void AddParticle(FEMSystem & system, glm::vec3 const & pos) {
        system.positions.push_back(pos);
        system.velocities.emplace_back(0.0f);
        system.masses.push_back(0.0f); // assign mass later
        system.fixed.push_back(false);
        system.forces.emplace_back(0.0f);
        system.externalForces.emplace_back(0.0f);
    }

    static void AddTet(FEMSystem & system, int v0, int v1, int v2, int v3) {
        system.tets.emplace_back(v0, v1, v2, v3);
    }

    static void AddGridCellTets(
        FEMSystem & system,
        int v000,
        int v001,
        int v010,
        int v011,
        int v100,
        int v101,
        int v110,
        int v111) {
        AddTet(system, v000, v001, v011, v111);
        AddTet(system, v000, v010, v011, v111);
        AddTet(system, v000, v001, v101, v111);
        AddTet(system, v000, v100, v101, v111);
        AddTet(system, v000, v010, v110, v111);
        AddTet(system, v000, v100, v110, v111);
    }

    static void BuildSurfaceFaces(FEMSystem & system) {
        system.surfaceFaces.clear();

        std::map<std::array<int, 3>, std::array<int, 3>> boundaryFaces;
        for (auto const & tet : system.tets) {
            std::array<std::array<int, 3>, 4> const faces = {{
                { tet.indices[0], tet.indices[2], tet.indices[1] },
                { tet.indices[0], tet.indices[1], tet.indices[3] },
                { tet.indices[0], tet.indices[3], tet.indices[2] },
                { tet.indices[1], tet.indices[2], tet.indices[3] },
            }};

            for (auto const & face : faces) {
                std::array<int, 3> key = face;
                std::sort(key.begin(), key.end());
                auto it = boundaryFaces.find(key);
                if (it == boundaryFaces.end())
                    boundaryFaces.emplace(key, face);
                else
                    boundaryFaces.erase(it);
            }
        }

        system.surfaceFaces.reserve(boundaryFaces.size());
        for (auto const & [_, face] : boundaryFaces)
            system.surfaceFaces.push_back(face);
    }

    static void BuildGridBlock(FEMSystem & system) {
        // create grid of particles
        for (std::size_t i = 0; i <= system.wx; ++i)
            for (std::size_t j = 0; j <= system.wy; ++j)
                for (std::size_t k = 0; k <= system.wz; ++k)
                    AddParticle(system, { i * system.delta, j * system.delta, k * system.delta });

        // create tetrahedral elements
        for (std::size_t i = 0; i < system.wx; i++)
            for (std::size_t j = 0; j < system.wy; j++)
                for (std::size_t k = 0; k < system.wz; k++) {
                    AddGridCellTets(system,
                        system.GetID(i, j, k),
                        system.GetID(i, j, k + 1),
                        system.GetID(i, j + 1, k),
                        system.GetID(i, j + 1, k + 1),
                        system.GetID(i + 1, j, k),
                        system.GetID(i + 1, j, k + 1),
                        system.GetID(i + 1, j + 1, k),
                        system.GetID(i + 1, j + 1, k + 1));
                }

        for (std::size_t j = 0; j <= system.wy; j++)
            for (std::size_t k = 0; k <= system.wz; k++)
                system.fixed[system.GetID(0, j, k)] = true;
    }

    static void BuildSphere(FEMSystem & system) {
        std::vector<int> vertexMap((system.wx + 1) * (system.wy + 1) * (system.wz + 1), -1);
        glm::vec3 const center(
            0.5f * static_cast<float>(system.wx) * system.delta,
            0.5f * static_cast<float>(system.wy) * system.delta,
            0.5f * static_cast<float>(system.wz) * system.delta
        );
        float const radius = 0.5f * static_cast<float>(std::min({ system.wx, system.wy, system.wz })) * system.delta;
        float const radius2 = radius * radius;

        auto vertex = [&](std::size_t i, std::size_t j, std::size_t k) {
            int const oldId = system.GetID(i, j, k);
            int & newId = vertexMap[oldId];
            if (newId < 0) {
                newId = static_cast<int>(system.positions.size());
                AddParticle(system, { i * system.delta, j * system.delta, k * system.delta });
            }
            return newId;
        };

        for (std::size_t i = 0; i < system.wx; i++)
            for (std::size_t j = 0; j < system.wy; j++)
                for (std::size_t k = 0; k < system.wz; k++) {
                    glm::vec3 const cellCenter(
                        (static_cast<float>(i) + 0.5f) * system.delta,
                        (static_cast<float>(j) + 0.5f) * system.delta,
                        (static_cast<float>(k) + 0.5f) * system.delta
                    );
                    glm::vec3 const offset = cellCenter - center;
                    if (glm::dot(offset, offset) > radius2)
                        continue;

                    AddGridCellTets(system,
                        vertex(i, j, k),
                        vertex(i, j, k + 1),
                        vertex(i, j + 1, k),
                        vertex(i, j + 1, k + 1),
                        vertex(i + 1, j, k),
                        vertex(i + 1, j, k + 1),
                        vertex(i + 1, j + 1, k),
                        vertex(i + 1, j + 1, k + 1));
                }
    }

    static bool InsideEllipsoid(glm::vec3 const & p, glm::vec3 const & center, glm::vec3 const & radius) {
        glm::vec3 const d(
            (p.x - center.x) / radius.x,
            (p.y - center.y) / radius.y,
            (p.z - center.z) / radius.z
        );
        return glm::dot(d, d) <= 1.0f;
    }

    static bool InsideTeddyBear(glm::vec3 const & p) {
        return
            InsideEllipsoid(p, glm::vec3( 0.00f, -0.16f,  0.00f), glm::vec3(0.22f, 0.26f, 0.17f)) ||
            InsideEllipsoid(p, glm::vec3( 0.00f,  0.18f,  0.00f), glm::vec3(0.17f, 0.16f, 0.14f)) ||
            InsideEllipsoid(p, glm::vec3(-0.15f,  0.34f,  0.00f), glm::vec3(0.08f, 0.08f, 0.07f)) ||
            InsideEllipsoid(p, glm::vec3( 0.15f,  0.34f,  0.00f), glm::vec3(0.08f, 0.08f, 0.07f)) ||
            InsideEllipsoid(p, glm::vec3(-0.28f, -0.09f,  0.00f), glm::vec3(0.07f, 0.18f, 0.10f)) ||
            InsideEllipsoid(p, glm::vec3( 0.28f, -0.09f,  0.00f), glm::vec3(0.07f, 0.18f, 0.10f)) ||
            InsideEllipsoid(p, glm::vec3(-0.10f, -0.40f,  0.00f), glm::vec3(0.08f, 0.10f, 0.10f)) ||
            InsideEllipsoid(p, glm::vec3( 0.10f, -0.40f,  0.00f), glm::vec3(0.08f, 0.10f, 0.10f)) ||
            InsideEllipsoid(p, glm::vec3( 0.00f,  0.14f,  0.15f), glm::vec3(0.08f, 0.05f, 0.07f));
    }

    static void BuildTeddyBear(FEMSystem & system) {
        std::vector<int> vertexMap((system.wx + 1) * (system.wy + 1) * (system.wz + 1), -1);
        glm::vec3 const center(
            0.5f * static_cast<float>(system.wx) * system.delta,
            0.5f * static_cast<float>(system.wy) * system.delta,
            0.5f * static_cast<float>(system.wz) * system.delta
        );
        float const scale = static_cast<float>(std::min({ system.wx, system.wy, system.wz })) * system.delta;

        auto vertex = [&](std::size_t i, std::size_t j, std::size_t k) {
            int const oldId = system.GetID(i, j, k);
            int & newId = vertexMap[oldId];
            if (newId < 0) {
                newId = static_cast<int>(system.positions.size());
                AddParticle(system, { i * system.delta, j * system.delta, k * system.delta });
            }
            return newId;
        };

        for (std::size_t i = 0; i < system.wx; i++)
            for (std::size_t j = 0; j < system.wy; j++)
                for (std::size_t k = 0; k < system.wz; k++) {
                    glm::vec3 const cellCenter(
                        (static_cast<float>(i) + 0.5f) * system.delta,
                        (static_cast<float>(j) + 0.5f) * system.delta,
                        (static_cast<float>(k) + 0.5f) * system.delta
                    );
                    glm::vec3 const p = (cellCenter - center) / scale;
                    if (! InsideTeddyBear(p))
                        continue;

                    AddGridCellTets(system,
                        vertex(i, j, k),
                        vertex(i, j, k + 1),
                        vertex(i, j + 1, k),
                        vertex(i, j + 1, k + 1),
                        vertex(i + 1, j, k),
                        vertex(i + 1, j, k + 1),
                        vertex(i + 1, j + 1, k),
                        vertex(i + 1, j + 1, k + 1));
                }
    }

    void BuildSoftBodyStructure(FEMSystem & system, SoftBodyType type) {
        ClearStructure(system);

        switch (type) {
        case SoftBodyType::GridBlock:
            BuildGridBlock(system);
            break;
        case SoftBodyType::Sphere:
            BuildSphere(system);
            break;
        case SoftBodyType::TeddyBear:
            BuildTeddyBear(system);
            break;
        }

        BuildSurfaceFaces(system);
    }

} // namespace VCX::Labs::FEM
