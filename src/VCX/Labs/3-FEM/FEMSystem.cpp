#include "Integrator.h"
#include "FEMSystem.h"

#include <cmath>

#include <glm/geometric.hpp>

namespace VCX::Labs::FEM {

    static glm::vec3 ClosestPointOnTriangle(
        glm::vec3 const & p,
        glm::vec3 const & a,
        glm::vec3 const & b,
        glm::vec3 const & c,
        glm::vec3 & bary) {
        glm::vec3 const ab = b - a;
        glm::vec3 const ac = c - a;
        glm::vec3 const ap = p - a;
        float const d1 = glm::dot(ab, ap);
        float const d2 = glm::dot(ac, ap);
        if (d1 <= 0.0f && d2 <= 0.0f) {
            bary = glm::vec3(1.0f, 0.0f, 0.0f);
            return a;
        }

        glm::vec3 const bp = p - b;
        float const d3 = glm::dot(ab, bp);
        float const d4 = glm::dot(ac, bp);
        if (d3 >= 0.0f && d4 <= d3) {
            bary = glm::vec3(0.0f, 1.0f, 0.0f);
            return b;
        }

        float const vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
            float const v = d1 / (d1 - d3);
            bary = glm::vec3(1.0f - v, v, 0.0f);
            return a + v * ab;
        }

        glm::vec3 const cp = p - c;
        float const d5 = glm::dot(ab, cp);
        float const d6 = glm::dot(ac, cp);
        if (d6 >= 0.0f && d5 <= d6) {
            bary = glm::vec3(0.0f, 0.0f, 1.0f);
            return c;
        }

        float const vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
            float const w = d2 / (d2 - d6);
            bary = glm::vec3(1.0f - w, 0.0f, w);
            return a + w * ac;
        }

        float const va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
            float const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            bary = glm::vec3(0.0f, 1.0f - w, w);
            return b + w * (c - b);
        }

        float const denom = 1.0f / (va + vb + vc);
        float const v = vb * denom;
        float const w = vc * denom;
        bary = glm::vec3(1.0f - v - w, v, w);
        return a + ab * v + ac * w;
    }

    FEMSystem::FEMSystem() {
        model = std::make_unique<LinearModel>();
        integrator = std::make_unique<ExplicitIntegrator>();
        ResetSystem();
    }

    FEMSystem::~FEMSystem() = default;

    void FEMSystem::ResetSystem() {
        SetupLameParameters();
        BuildSoftBodyStructure(*this, softBodyType);

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

    void FEMSystem::ApplyCollisionForces() {
        if (!enableCollision)
            return;

        if (useSphereCollider) {
            for (auto const & face : surfaceFaces) {
                int const i0 = face[0];
                int const i1 = face[1];
                int const i2 = face[2];
                glm::vec3 bary(0.0f);
                glm::vec3 const q = ClosestPointOnTriangle(
                    sphereCenter,
                    positions[i0],
                    positions[i1],
                    positions[i2],
                    bary
                );
                glm::vec3 const dir = q - sphereCenter;
                float const dist = glm::length(dir);
                float const penetration = sphereRadius - dist;
                if (penetration <= 0.0f)
                    continue;

                glm::vec3 const normal = dist > 1e-6f ? dir / dist : glm::vec3(0.0f, 1.0f, 0.0f);
                glm::vec3 const qVelocity =
                    bary.x * velocities[i0] +
                    bary.y * velocities[i1] +
                    bary.z * velocities[i2];
                float const normalVelocity = glm::dot(qVelocity, normal);
                glm::vec3 collisionForce = collisionStiffness * penetration * normal;
                if (normalVelocity < 0.0f)
                    collisionForce += -collisionDamping * normalVelocity * normal;

                if (!fixed[i0]) forces[i0] += bary.x * collisionForce;
                if (!fixed[i1]) forces[i1] += bary.y * collisionForce;
                if (!fixed[i2]) forces[i2] += bary.z * collisionForce;
            }
            return;
        }

        for (std::size_t i = 0; i < positions.size(); ++i) {
            if (fixed[i])
                continue;

            glm::vec3 normal(0.0f, 1.0f, 0.0f);
            float penetration = 0.0f;

            if (useSphereCollider) {
                glm::vec3 const dir = positions[i] - sphereCenter;
                float const dist = glm::length(dir);
                penetration = sphereRadius - dist;
                if (penetration <= 0.0f)
                    continue;
                normal = dist > 1e-6f ? dir / dist : glm::vec3(0.0f, 1.0f, 0.0f);
            } else {
                penetration = groundY - positions[i].y;
                if (penetration <= 0.0f)
                    continue;
            }

            if (penetration <= 0.0f)
                continue;

            float const normalVelocity = glm::dot(velocities[i], normal);
            glm::vec3 collisionForce = collisionStiffness * penetration * normal;
            if (normalVelocity < 0.0f)
                collisionForce += -collisionDamping * normalVelocity * normal;

            forces[i] += collisionForce;
        }
    }
} // namespace VCX::Labs::FEM
