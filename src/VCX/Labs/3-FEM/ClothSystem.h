#pragma once

#include <array>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <glm/glm.hpp>

namespace VCX::Labs::FEM {

    struct ClothElement {
        std::array<int, 3> indices;
        Eigen::Matrix2f    DmInv;
        float              area;

        ClothElement(int v0, int v1, int v2) : indices({v0, v1, v2}), area(0.0f) {
            DmInv.setIdentity();
        }
    };

    class MembraneModel {
    public:
        virtual Eigen::Matrix<float, 3, 2> ComputeP(const Eigen::Matrix<float, 3, 2>& F, float mu, float lambda) = 0;
        virtual ~MembraneModel() = default;
    };

    class MembraneStVK : public MembraneModel {
    public:
        Eigen::Matrix<float, 3, 2> ComputeP(const Eigen::Matrix<float, 3, 2>& F, float mu, float lambda) override;
    };

    class MembraneNeoHookean : public MembraneModel {
    public:
        Eigen::Matrix<float, 3, 2> ComputeP(const Eigen::Matrix<float, 3, 2>& F, float mu, float lambda) override;
    };

    class ClothSystem {
    public:
        std::vector<glm::vec3>  positions;
        std::vector<glm::vec3>  velocities;
        std::vector<float>      masses;
        std::vector<bool>       fixed;
        std::vector<glm::vec3>  forces;
        std::vector<glm::vec3>  externalForces;

        std::vector<ClothElement> elements;
        std::vector<std::array<int, 3>> triangles;

        std::unique_ptr<MembraneModel> model;

        float E = 5000.0f;
        float nu = 0.3f;
        float mu = 0.0f;
        float lambda = 0.0f;
        float density = 0.5f;

        glm::vec3 gravity = glm::vec3(0.0f, -9.8f, 0.0f);
        float damping = 1.0f;

        bool enableCollision = true;
        float groundY = -3.0f;

        int wx = 20;
        int wy = 20;
        float dx = 0.2f;
        float dy = 0.2f;

        ClothSystem();
        void ResetSystem();
        void Update(float dt);
        void SetupLameParameters();
        void ComputeInternalForces();

        inline int GetID(int i, int j) const {
            return i * (wy + 1) + j;
        }

    private:
        void AddParticle(const glm::vec3& pos);
        void AddElement(int v0, int v1, int v2);
    };
} // namespace VCX::Labs::FEM