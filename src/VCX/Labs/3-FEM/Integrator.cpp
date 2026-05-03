#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "Integrator.h"
#include "FEMSystem.h"

namespace VCX::Labs::FEM {

    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
        return v;
    }

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size())
        );
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    // solve Ax = b and return x
    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const & b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }

    // ported from vci lab4 implicit euler
    // some variables are now useless, but to keep the code highly consistent with the original one, I decided to keep them
    void ImplicitIntegrator::Step(FEMSystem& system, float dt) {
        system.SetupLameParameters();

        int const steps = 1;
        float const h = dt / steps; 
        for (std::size_t s = 0; s < steps; s++) {
            int n = static_cast<int>(system.positions.size());
            Eigen::VectorXf x0=glm2eigen(system.positions);
            Eigen::VectorXf v0=glm2eigen(system.velocities);
            Eigen::VectorXf gravity(n * 3);
            gravity.setZero();
            for (int i = 0; i < n; i++)
                if (!system.fixed[i]) {
                    glm::vec3 const extAccel = system.externalForces[i] / system.masses[i];
                    gravity[3*i+0]=system.gravity.x + extAccel.x;
                    gravity[3*i+1]=system.gravity.y + extAccel.y;
                    gravity[3*i+2]=system.gravity.z + extAccel.z;
                }
            Eigen::VectorXf y=x0+h*v0+h*h*gravity;//x_{n+1}=y+h^2/M*f_{int}(x_{n+1}),f_ext/M=gravity
            Eigen::VectorXf x_next=x0;
            int iter_max=this->maxIters;           
            for(int iter=0;iter<iter_max;iter++){
                Eigen::VectorXf f_int(n * 3);
                f_int.setZero();
                std::vector<Eigen::Triplet<float>> triplets;
                for(auto const & tet:system.tets){
                    int p[4]={tet.indices[0],tet.indices[1],tet.indices[2],tet.indices[3]};
                    glm::vec3 cur_pos[4];
                    for(int i=0;i<4;++i)
                        cur_pos[i]=glm::vec3(x_next[3*p[i]+0],x_next[3*p[i]+1],x_next[3*p[i]+2]);
                    Eigen::Matrix3f Ds;
                    Ds.col(0) = Eigen::Vector3f(cur_pos[1].x - cur_pos[0].x, cur_pos[1].y - cur_pos[0].y, cur_pos[1].z - cur_pos[0].z);
                    Ds.col(1) = Eigen::Vector3f(cur_pos[2].x - cur_pos[0].x, cur_pos[2].y - cur_pos[0].y, cur_pos[2].z - cur_pos[0].z);
                    Ds.col(2) = Eigen::Vector3f(cur_pos[3].x - cur_pos[0].x, cur_pos[3].y - cur_pos[0].y, cur_pos[3].z - cur_pos[0].z);
                    Eigen::Matrix3f F=Ds*tet.E_Inv;
                    Eigen::Matrix3f P=system.model->ComputeP(F,system.mu,system.lambda);
                    Eigen::Matrix3f H=-tet.volume*P*tet.E_Inv.transpose();
                    glm::vec3 f1(H(0,0), H(1,0), H(2,0));
                    glm::vec3 f2(H(0,1), H(1,1), H(2,1));
                    glm::vec3 f3(H(0,2), H(1,2), H(2,2));
                    glm::vec3 f0 = -f1 - f2 - f3;
                    if (!system.fixed[p[0]]) f_int.segment<3>(3 * p[0]) += Eigen::Vector3f(f0.x, f0.y, f0.z);
                    if (!system.fixed[p[1]]) f_int.segment<3>(3 * p[1]) += Eigen::Vector3f(f1.x, f1.y, f1.z);
                    if (!system.fixed[p[2]]) f_int.segment<3>(3 * p[2]) += Eigen::Vector3f(f2.x, f2.y, f2.z);
                    if (!system.fixed[p[3]]) f_int.segment<3>(3 * p[3]) += Eigen::Vector3f(f3.x, f3.y, f3.z);
                    //compute hessian of FEM energy
                    Eigen::Matrix<float, 12, 12> Ke=system.model->ComputeK(F, system.mu, system.lambda, tet.volume, tet.E_Inv);
                    for(int a=0;a<4;++a)
                        for(int b=0;b<4;++b){
                            if(!system.fixed[p[a]]||!system.fixed[p[b]]){
                                for(int i=0; i<3; ++i)
                                    for(int j=0; j<3; ++j)
                                        triplets.emplace_back(3*p[a]+i, 3*p[b]+j, Ke(3*a+i, 3*b+j));
                            }
                        }
                }
                //Eigen::VectorXf nabla_g=x_next-y-h*h*f_int/system.Mass;//∇g(x)
                Eigen::VectorXf nabla_g=x_next-y;
                for (int i = 0; i < n; i++)
                    if (!system.fixed[i])
                        nabla_g.segment<3>(3*i)-=h*h*f_int.segment<3>(3*i)/system.masses[i];
                auto H_E=CreateEigenSparseMatrix(n * 3, triplets);
                Eigen::SparseMatrix<float> nabla2_g=Eigen::SparseMatrix<float>(n * 3, n * 3);
                nabla2_g.setIdentity();
                //nabla2_g+=h*h*H_E/system.Mass;//H=I+h^2/M*H_E
                for (int m=0;m<H_E.outerSize();++m)
                    for (Eigen::SparseMatrix<float>::InnerIterator it(H_E,m);it;++it){
                        int row_idx=it.row();
                        int node_idx=row_idx/3;
                        nabla2_g.coeffRef(it.row(),it.col())+=h*h*it.value()/system.masses[node_idx];
                    }
                Eigen::VectorXf dx=ComputeSimplicialLLT(nabla2_g,-nabla_g);
                for(int i=0;i<n;i++)
                    if(system.fixed[i])
                        dx.segment<3>(3*i).setZero();
                x_next+=dx;
                if (dx.norm() < this->tol) break;
            }
            Eigen::VectorXf v_next=(x_next - x0)/h;
            system.positions=eigen2glm(x_next);
            system.velocities=eigen2glm(v_next);
        }
    }

    void ExplicitIntegrator::Step(FEMSystem& system, float dt) {
        system.SetupLameParameters();
        system.ComputeInternalForces();
        for (std::size_t i = 0; i < system.positions.size(); ++i) {
            if(system.fixed[i]) continue;
            glm::vec3 f_ext = system.masses[i] * system.gravity - system.damping * system.velocities[i] + system.externalForces[i];
            glm::vec3 f_tot = system.forces[i]  + f_ext;

            system.velocities[i] += (f_tot / system.masses[i]) * dt;
            system.positions[i] += system.velocities[i] * dt;
        }
    }
} //namespace VCX::Labs::FEM