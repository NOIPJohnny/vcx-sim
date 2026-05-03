#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace VCX::Labs::FEM {

    class FEMSystem;

    class TimeIntegrator {
    public:
        virtual ~TimeIntegrator() = default;
        virtual void Step(FEMSystem& system, float dt) = 0;
    };

    class ExplicitIntegrator : public TimeIntegrator {
    public:
        void Step(FEMSystem& system, float dt) override;
    };

    class ImplicitIntegrator : public TimeIntegrator {
    public:
        int maxIters = 8;
        float tol = 1e-4f;
        void Step(FEMSystem& system, float dt) override;
    };
} //namespace VCX::Labs::FEM