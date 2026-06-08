#pragma once

#include <array>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <glm/glm.hpp>

#include "FEMElement.h"
#include "FEMSoftBodyBuilder.h"
#include "HyperElasticModels.h"

namespace VCX::Labs::FEM {

    class TimeIntegrator;

    class FEMSystem {
    public:
        std::vector<glm::vec3>  positions; // current positions of vertices
        std::vector<glm::vec3>  velocities;
        std::vector<float>      masses;
        std::vector<bool>       fixed;
        std::vector<glm::vec3>  forces;
        std::vector<glm::vec3>  externalForces;

        std::vector<TetElement> tets;
        std::vector<std::array<int, 3>> surfaceFaces;

        std::unique_ptr<HyperElasticModel> model;

        std::unique_ptr<TimeIntegrator> integrator;

        float E = 20000; // Youngs Modulus
        float nu = 0.2f; // poisson ratio
        float mu = 0.0f; // lame parameter 1
        float lambda = 0.0f; // lame parameter 2
        float density = 400.0f;
        
        glm::vec3 gravity = glm::vec3(0.0f, -0.05f, 0.0f);
        float damping = 0.5f;

        bool enableCollision = true;
        bool useSphereCollider = false;
        float groundY = 0.0f;
        glm::vec3 sphereCenter = glm::vec3(5.0f, -0.6f, 1.0f);
        float sphereRadius = 1.0f;
        float collisionStiffness = 100000.0f;
        float collisionDamping = 500.0f;

        // grid
        SoftBodyType softBodyType = SoftBodyType::GridBlock;
        std::size_t wx = 8, wy = 2, wz = 2;
        float delta = 1.0f; // grid spacing

        FEMSystem();
        ~FEMSystem();
        void ResetSystem();
        void Update(float dt);
        void SetupLameParameters();
        void ComputeInternalForces();
        void ApplyCollisionForces();

        inline int GetID(std::size_t const i, std::size_t const j, std::size_t const k) const {
            return i * (wy + 1) * (wz + 1) + j * (wz + 1) + k;
        }
    };
} // namespace VCX::Labs::FEM
