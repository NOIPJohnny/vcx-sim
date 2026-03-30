#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace VCX::Labs::RigidBody {

    class RigidBodySystem;

    class RigidBodyItem {
    public:

        friend class RigidBodySystem;
        
        RigidBodyItem(
            const Eigen::Vector3f& dim,
            float mass,
            const Eigen::Vector3f& position = Eigen::Vector3f::Zero(),
            const Eigen::Quaternionf& orientation = Eigen::Quaternionf::Identity(),
            const Eigen::Vector3f& v = Eigen::Vector3f::Zero(),
            const Eigen::Vector3f& w = Eigen::Vector3f::Zero()
        );

        const Eigen::Vector3f& GetDim() const { return dim; }
        const float GetMass() const { return mass; }
        const float GetInvMass() const { return mass_inv; }
        const Eigen::Vector3f& GetPosition() const { return position; }
        const Eigen::Quaternionf& GetOrient() const { return orientation; }
        const Eigen::Vector3f& Getv() const { return v; }
        const Eigen::Vector3f& Getw() const { return w; }


        void ClearForces();
        void ApplyForce(const Eigen::Vector3f& f); // apply force at the center of mass
        void ApplyForceAtPoint(const Eigen::Vector3f& f, const Eigen::Vector3f& p);
        void PositionShift(const Eigen::Vector3f& shift) { position += shift; }
        void vShift(const Eigen::Vector3f& dv) { v += dv; }
        void wShift(const Eigen::Vector3f& dw) { w += dw; }
        Eigen::Matrix3f GetInertiaTensorWorld() const;
        Eigen::Matrix3f GetInvInertiaTensorWorld() const;


    private:
        Eigen::Vector3f    dim;  // length, width, height
        float              mass;
        float              mass_inv;
        Eigen::Matrix3f    I_0;      // inertia tensor
        Eigen::Matrix3f    I_0_inv;

        Eigen::Vector3f    position;  // center of mass position
        Eigen::Quaternionf orientation;
        Eigen::Vector3f    v;
        Eigen::Vector3f    w;

        Eigen::Vector3f    force;
        Eigen::Vector3f    torque;
    };
} // namespace VCX::Labs::RigidBody