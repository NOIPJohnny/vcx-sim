#include "Integrator.h"
#include "FEMSystem.h"

namespace VCX::Labs::FEM {

    FEMSystem::FEMSystem() {
        model = std::make_unique<LinearModel>();
        integrator = std::make_unique<ExplicitIntegrator>();
        ResetSystem();
    }

    FEMSystem::~FEMSystem() = default;

    void FEMSystem::ResetSystem() {
        positions.clear();
        velocities.clear();
        masses.clear();
        fixed.clear();
        forces.clear();
        externalForces.clear();
        tets.clear();

        SetupLameParameters();

        // create grid of particles
        for(std::size_t i = 0; i <= wx; ++i)
            for(std::size_t j = 0; j <= wy; ++j)
                for(std::size_t k = 0; k <= wz; ++k) {
                    AddParticle({i * delta, j * delta, k * delta});
                }

        // create tetrahedral elements
        for (std::size_t i = 0; i < wx; i++)
            for (std::size_t j = 0; j < wy; j++)
                for (std::size_t k = 0; k < wz; k++) {
                    AddTet(GetID(i, j, k),   
                        GetID(i, j, k + 1), 
                        GetID(i, j + 1, k + 1), 
                        GetID(i + 1, j + 1, k + 1));
                    AddTet(GetID(i, j, k),   
                        GetID(i, j + 1, k), 
                        GetID(i, j + 1, k + 1), 
                        GetID(i + 1, j + 1, k + 1));
                    AddTet(GetID(i, j, k),   
                        GetID(i, j, k + 1), 
                        GetID(i + 1, j, k + 1), 
                        GetID(i + 1, j + 1, k + 1));  
                    AddTet(GetID(i, j, k), 
                        GetID(i + 1, j, k), 
                        GetID(i + 1, j, k + 1), 
                        GetID(i + 1, j + 1, k + 1));  
                    AddTet(GetID(i, j, k),
                        GetID(i, j + 1, k), 
                        GetID(i + 1, j + 1, k), 
                        GetID(i + 1, j + 1, k + 1));
                    AddTet(GetID(i, j, k),   
                        GetID(i + 1, j, k), 
                        GetID(i + 1, j + 1, k), 
                        GetID(i + 1, j + 1, k + 1));
                }
            
    for (std::size_t j = 0; j <= wy; j++)
        for (std::size_t k = 0; k <= wz; k++)
            fixed[GetID(0, j, k)] = true;

    for (auto& tet : tets) {
            glm::vec3 x0 = positions[tet.indices[0]];
            glm::vec3 x1 = positions[tet.indices[1]];
            glm::vec3 x2 = positions[tet.indices[2]];
            glm::vec3 x3 = positions[tet.indices[3]];

            Eigen::Matrix3f Dm; // Reference Shape: E in slides
            Dm.col(0) = Eigen::Vector3f(x1.x - x0.x, x1.y - x0.y, x1.z - x0.z);
            Dm.col(1) = Eigen::Vector3f(x2.x - x0.x, x2.y - x0.y, x2.z - x0.z);
            Dm.col(2) = Eigen::Vector3f(x3.x - x0.x, x3.y - x0.y, x3.z - x0.z);

            tet.E_Inv = Dm.inverse();
            tet.volume = std::abs(Dm.determinant()) / 6.0f;

            float tetMass = density * tet.volume;
            for (int i = 0; i < 4; i++)
                masses[tet.indices[i]] += tetMass / 4.0f;
        }
    }

    void FEMSystem::SetupLameParameters() {
        mu = E / (2.0f * (1.0f + nu));
        lambda = E * nu / ((1.0f + nu) * (1.0f - 2.0f * nu));
    }

    void FEMSystem::Update(float dt) {
        if (integrator)
            integrator->Step(*this, dt);
    }

    void FEMSystem::AddParticle(const glm::vec3& pos) {
        positions.push_back(pos);
        velocities.emplace_back(0.0f);
        masses.push_back(0.0f); // assign mass later
        fixed.push_back(false);
        forces.emplace_back(0.0f);
        externalForces.emplace_back(0.0f);
    }

    void FEMSystem::AddTet(int v0, int v1, int v2, int v3) {
        tets.emplace_back(v0, v1, v2, v3);
    }

    void FEMSystem::ComputeInternalForces() {
        for (auto& force : forces)
            force = glm::vec3(0.0f);

        for (const auto& tet : tets) {
            glm::vec3 x0 = positions[tet.indices[0]];
            glm::vec3 x1 = positions[tet.indices[1]];
            glm::vec3 x2 = positions[tet.indices[2]];
            glm::vec3 x3 = positions[tet.indices[3]];

            Eigen::Matrix3f Ds; // Deformed Shape
            Ds.col(0) = Eigen::Vector3f(x1.x - x0.x, x1.y - x0.y, x1.z - x0.z);
            Ds.col(1) = Eigen::Vector3f(x2.x - x0.x, x2.y - x0.y, x2.z - x0.z);
            Ds.col(2) = Eigen::Vector3f(x3.x - x0.x, x3.y - x0.y, x3.z - x0.z);

            Eigen::Matrix3f F = Ds * tet.E_Inv;
            Eigen::Matrix3f P = model->ComputeP(F, mu, lambda);

            Eigen::Matrix3f H = -tet.volume * P * tet.E_Inv.transpose();

            // Distribute forces to vertices
            glm::vec3 f1(H(0, 0), H(1, 0), H(2, 0));
            glm::vec3 f2(H(0, 1), H(1, 1), H(2, 1));
            glm::vec3 f3(H(0, 2), H(1, 2), H(2, 2));

            glm::vec3 f0 = -f1 - f2 - f3;

            if (!fixed[tet.indices[0]]) forces[tet.indices[0]] += f0;
            if (!fixed[tet.indices[1]]) forces[tet.indices[1]] += f1;
            if (!fixed[tet.indices[2]]) forces[tet.indices[2]] += f2;
            if (!fixed[tet.indices[3]]) forces[tet.indices[3]] += f3;
        }
    }
} // namespace VCX::Labs::FEM