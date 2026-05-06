#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/SVD>

namespace VCX::Labs::FEM {

    inline Eigen::Matrix3f ExtractRotation(const Eigen::Matrix3f& F) {
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f U = svd.matrixU();
        Eigen::Matrix3f V = svd.matrixV();
        Eigen::Matrix3f R = U * V.transpose();
        if (R.determinant() < 0.0f) {
            U.col(2) *= -1.0f;
            R = U * V.transpose();
        }
        return R;
    }

    class HyperElasticModel {
    public:
        virtual Eigen::Matrix3f ComputeP(const Eigen::Matrix3f& F, float mu, float lambda) = 0;
        virtual ~HyperElasticModel() = default;

        // a universal implementation for finite difference
        virtual Eigen::Matrix<float, 12, 12> ComputeK(const Eigen::Matrix3f& F, float mu, float lambda, float volume, const Eigen::Matrix3f& beta) {
            const float h = 1e-4f; // disturbulence
            Eigen::Matrix<float, 12, 12> Ke;
            Ke.setZero();
            Eigen::Matrix3f Dm = beta.inverse();
            Eigen::Matrix3f Ds = F * Dm;
            Eigen::Matrix3f P0 = ComputeP(F, mu, lambda);
            Eigen::Matrix3f H0 = -volume * P0 * beta.transpose();
            Eigen::Vector3f f0_orig[4];
            f0_orig[1] = H0.col(0);
            f0_orig[2] = H0.col(1);
            f0_orig[3] = H0.col(2);
            f0_orig[0] = -(f0_orig[1] + f0_orig[2] + f0_orig[3]);
            for (int b = 1; b <= 3; ++b)
                for (int j = 0; j < 3; ++j) {
                    Eigen::Matrix3f Ds_alt = Ds;
                    Ds_alt(j, b - 1) += h;
                    Eigen::Matrix3f F_alt  = Ds_alt * beta;
                    Eigen::Matrix3f P_alt  = ComputeP(F_alt, mu, lambda);
                    Eigen::Matrix3f H_alt  = -volume * P_alt * beta.transpose();
                    Eigen::Vector3f f_alt[4];
                    f_alt[1] = H_alt.col(0);
                    f_alt[2] = H_alt.col(1);
                    f_alt[3] = H_alt.col(2);
                    f_alt[0] = -(f_alt[1] + f_alt[2] + f_alt[3]);
                    for (int a = 0; a < 4; ++a)
                        Ke.block<3, 1>(3 * a, 3 * b + j) =
                            (f0_orig[a] - f_alt[a]) / h;
                }
            for (int a = 0; a < 4; ++a)
                for (int j = 0; j < 3; ++j)
                    Ke.block<3, 1>(3 * a, j) = -(
                        Ke.block<3, 1>(3 * a, 3 + j) + 
                        Ke.block<3, 1>(3 * a, 6 + j) + 
                        Ke.block<3, 1>(3 * a, 9 + j)
                    );
            return Ke;
        }
    };

    class LinearModel : public HyperElasticModel {
    public:
        Eigen::Matrix3f ComputeP(const Eigen::Matrix3f& F, float mu, float lambda) override {
            /*G = \frac{1}{2}((I+\nabla u)^T(I+\nabla u) - I)
                = \frac{1}{2}(\nabla u + \nabla u^T + \nabla u^T \nabla u)
                = \frac{1}{2}(\nabla u + \nabla u^T) + \frac{1}{2}\nabla u^T \nabla u
                \approx \frac{1}{2}(\nabla u + \nabla u^T)   (neglecting the second order term)
                = \frac{1}{2}(F + F^T) - I       */
            Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
            Eigen::Matrix3f epsilon = 0.5f * (F + F.transpose()) - I;
            
            float trace_eps = epsilon.trace();
            Eigen::Matrix3f P = 2.0f * mu * epsilon + lambda * trace_eps * I; // F = I + \nabla u \\approx I
            return P;
        }
    };

    class StVKModel : public HyperElasticModel {
    public:
        Eigen::Matrix3f ComputeP(const Eigen::Matrix3f& F, float mu, float lambda) override {
            Eigen::Matrix3f const I = Eigen::Matrix3f::Identity();
            Eigen::Matrix3f const G = 0.5f * (F.transpose() * F - I);
            return F * (2.0f * mu * G + lambda * G.trace() * I);
        }
    };

    class NeoHookeanModel : public HyperElasticModel {
    public:
        Eigen::Matrix3f ComputeP(const Eigen::Matrix3f& F, float mu, float lambda) override {
            float const eps = 1e-6f;
            float const J = std::max(F.determinant(), eps);

            Eigen::Matrix3f cofactor;
            cofactor.col(0) = F.col(1).cross(F.col(2));
            cofactor.col(1) = F.col(2).cross(F.col(0));
            cofactor.col(2) = F.col(0).cross(F.col(1));
            Eigen::Matrix3f const FInvT = cofactor / J;

            return mu * (F - FInvT) + lambda * std::log(J) * FInvT;
        }
    };

    class CorotatedModel : public HyperElasticModel {
    public:
        Eigen::Matrix3f ComputeP(const Eigen::Matrix3f& F, float mu, float lambda) override {
            Eigen::Matrix3f const I = Eigen::Matrix3f::Identity();
            Eigen::Matrix3f const R = ExtractRotation(F);
            float const strainTrace = (R.transpose() * F - I).trace();
            return 2.0f * mu * (F - R) + lambda * strainTrace * R;
        }
    };
} // namespace VCX::Labs::FEM