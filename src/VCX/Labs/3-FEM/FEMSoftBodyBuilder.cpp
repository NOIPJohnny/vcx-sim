#include "FEMSoftBodyBuilder.h"
#include "FEMSystem.h"

#include <algorithm>
#include <array>
#include <map>

namespace VCX::Labs::FEM {

    int SoftBodyTypeCount() {
        return 1;
    }

    char const * SoftBodyTypeName(SoftBodyType type) {
        switch (type) {
        case SoftBodyType::GridBlock: return "Grid Block";
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
                    AddTet(system,
                        system.GetID(i, j, k),
                        system.GetID(i, j, k + 1),
                        system.GetID(i, j + 1, k + 1),
                        system.GetID(i + 1, j + 1, k + 1));
                    AddTet(system,
                        system.GetID(i, j, k),
                        system.GetID(i, j + 1, k),
                        system.GetID(i, j + 1, k + 1),
                        system.GetID(i + 1, j + 1, k + 1));
                    AddTet(system,
                        system.GetID(i, j, k),
                        system.GetID(i, j, k + 1),
                        system.GetID(i + 1, j, k + 1),
                        system.GetID(i + 1, j + 1, k + 1));
                    AddTet(system,
                        system.GetID(i, j, k),
                        system.GetID(i + 1, j, k),
                        system.GetID(i + 1, j, k + 1),
                        system.GetID(i + 1, j + 1, k + 1));
                    AddTet(system,
                        system.GetID(i, j, k),
                        system.GetID(i, j + 1, k),
                        system.GetID(i + 1, j + 1, k),
                        system.GetID(i + 1, j + 1, k + 1));
                    AddTet(system,
                        system.GetID(i, j, k),
                        system.GetID(i + 1, j, k),
                        system.GetID(i + 1, j + 1, k),
                        system.GetID(i + 1, j + 1, k + 1));
                }

        for (std::size_t j = 0; j <= system.wy; j++)
            for (std::size_t k = 0; k <= system.wz; k++)
                system.fixed[system.GetID(0, j, k)] = true;
    }

    void BuildSoftBodyStructure(FEMSystem & system, SoftBodyType type) {
        ClearStructure(system);

        switch (type) {
        case SoftBodyType::GridBlock:
            BuildGridBlock(system);
            break;
        }

        BuildSurfaceFaces(system);
    }

} // namespace VCX::Labs::FEM
