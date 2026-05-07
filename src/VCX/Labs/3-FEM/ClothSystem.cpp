#include "ClothSystem.h"
#include <glm/geometric.hpp>

namespace VCX::Labs::FEM {

    Eigen::Matrix<float, 3, 2> MembraneStVK::ComputeP(const Eigen::Matrix<float, 3, 2>& F, float mu, float lambda) {
        Eigen::Matrix2f I2 = Eigen::Matrix2f::Identity();
        Eigen::Matrix2f G = 0.5f * (F.transpose() * F - I2);
        return F * (2.0f * mu * G + lambda * G.trace() * I2);
    }

    Eigen::Matrix<float, 3, 2> MembraneNeoHookean::ComputeP(const Eigen::Matrix<float, 3, 2>& F, float mu, float lambda) {
        Eigen::Matrix2f C = F.transpose() * F;
        float J2 = std::sqrt(std::max(C.determinant(), 1e-6f));
        Eigen::Matrix2f Cinv = C.inverse();
        Eigen::Matrix<float, 3, 2> FinvT = F * Cinv;
        return mu * (F - FinvT) + lambda * std::log(J2) * FinvT;
    }

    ClothSystem::ClothSystem() {
        model = std::make_unique<MembraneStVK>();
        ResetSystem();
    }

    void ClothSystem::ResetSystem() {
        positions.clear();
        velocities.clear();
        masses.clear();
        fixed.clear();
        forces.clear();
        externalForces.clear();
        elements.clear();
        triangles.clear();

        SetupLameParameters();

        for (int i = 0; i <= wx; ++i)
            for (int j = 0; j <= wy; ++j)
                AddParticle(glm::vec3(i * dx, 0.0f - j * dy * 0.3f, j * dy));

        for (int i = 0; i < wx; ++i)
            for (int j = 0; j < wy; ++j) {
                int v00 = GetID(i, j);
                int v10 = GetID(i + 1, j);
                int v01 = GetID(i, j + 1);
                int v11 = GetID(i + 1, j + 1);
                AddElement(v00, v10, v11);
                AddElement(v00, v11, v01);
            }

        for (int j = 0; j <= wy; ++j)
            fixed[GetID(0, j)] = true;

        for (auto& elem : elements) {
            glm::vec3 X0 = positions[elem.indices[0]];
            glm::vec3 X1 = positions[elem.indices[1]];
            glm::vec3 X2 = positions[elem.indices[2]];

            glm::vec3 e1 = X1 - X0;
            glm::vec3 e2 = X2 - X0;
            float len1 = glm::length(e1);
            glm::vec3 d1 = e1 / len1;
            float dot12 = glm::dot(e2, d1);
            glm::vec3 proj = d1 * dot12;
            glm::vec3 perp = e2 - proj;
            float lenPerp = glm::length(perp);

            Eigen::Matrix2f Dm;
            Dm(0, 0) = len1;
            Dm(0, 1) = dot12;
            Dm(1, 0) = 0.0f;
            Dm(1, 1) = std::max(lenPerp, 1e-8f);

            elem.DmInv = Dm.inverse();
            elem.area = 0.5f * glm::length(glm::cross(e1, e2));
        }

        for (auto& elem : elements) {
            float elemMass = density * elem.area;
            for (int i = 0; i < 3; ++i)
                masses[elem.indices[i]] += elemMass / 3.0f;
        }

        for (auto& elem : elements)
            triangles.push_back({elem.indices[0], elem.indices[1], elem.indices[2]});
    }

    void ClothSystem::SetupLameParameters() {
        mu = E / (2.0f * (1.0f + nu));
        lambda = E * nu / ((1.0f + nu) * (1.0f - nu));
    }

    void ClothSystem::Update(float dt) {
        SetupLameParameters();
        ComputeInternalForces();

        for (size_t i = 0; i < positions.size(); ++i) {
            if (fixed[i]) continue;
            glm::vec3 f_ext = masses[i] * gravity - damping * velocities[i] + externalForces[i];
            glm::vec3 f_tot = forces[i] + f_ext;

            velocities[i] += (f_tot / std::max(masses[i], 1e-8f)) * dt;
            positions[i] += velocities[i] * dt;

            if (enableCollision && positions[i].y < groundY) {
                positions[i].y = groundY;
                if (velocities[i].y < 0.0f)
                    velocities[i].y = 0.0f;
            }
        }
    }

    void ClothSystem::ComputeInternalForces() {
        for (auto& f : forces)
            f = glm::vec3(0.0f);

        for (const auto& elem : elements) {
            glm::vec3 x0 = positions[elem.indices[0]];
            glm::vec3 x1 = positions[elem.indices[1]];
            glm::vec3 x2 = positions[elem.indices[2]];

            Eigen::Matrix<float, 3, 2> Ds;
            Ds.col(0) = Eigen::Vector3f(x1.x - x0.x, x1.y - x0.y, x1.z - x0.z);
            Ds.col(1) = Eigen::Vector3f(x2.x - x0.x, x2.y - x0.y, x2.z - x0.z);

            Eigen::Matrix<float, 3, 2> F = Ds * elem.DmInv;
            Eigen::Matrix<float, 3, 2> P = model->ComputeP(F, mu, lambda);

            Eigen::Matrix<float, 3, 2> H = -elem.area * P * elem.DmInv.transpose();

            glm::vec3 f1(H(0, 0), H(1, 0), H(2, 0));
            glm::vec3 f2(H(0, 1), H(1, 1), H(2, 1));
            glm::vec3 f0 = -f1 - f2;

            if (!fixed[elem.indices[0]]) forces[elem.indices[0]] += f0;
            if (!fixed[elem.indices[1]]) forces[elem.indices[1]] += f1;
            if (!fixed[elem.indices[2]]) forces[elem.indices[2]] += f2;
        }
    }

    void ClothSystem::AddParticle(const glm::vec3& pos) {
        positions.push_back(pos);
        velocities.emplace_back(0.0f);
        masses.push_back(0.0f);
        fixed.push_back(false);
        forces.emplace_back(0.0f);
        externalForces.emplace_back(0.0f);
    }

    void ClothSystem::AddElement(int v0, int v1, int v2) {
        elements.emplace_back(v0, v1, v2);
    }
} // namespace VCX::Labs::FEM