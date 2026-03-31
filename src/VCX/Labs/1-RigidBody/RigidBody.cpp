#include "RigidBody.h"

namespace VCX::Labs::RigidBody {

    RigidBodyItem::RigidBodyItem(
        RigidBodyType type,
        const Eigen::Vector3f& dim,
        float mass,
        const Eigen::Vector3f& position,
        const Eigen::Quaternionf& orientation,
        const Eigen::Vector3f& v,
        const Eigen::Vector3f& w
    ) : 
        type(RigidBodyType::Box),
        dim(dim),
        radius(0.0f),
        mass(mass),
        position(position),
        orientation(orientation),
        v(v),
        w(w),
        force(Eigen::Vector3f::Zero()),
        torque(Eigen::Vector3f::Zero()) 
    {
        if (mass > 0.0f) {
            mass_inv = 1.0f / mass;

            float dx = dim.x();
            float dy = dim.y();
            float dz = dim.z();

            I_0 = Eigen::Matrix3f::Zero();
            I_0(0, 0) = (1.0f / 12.0f) * mass * (dy * dy + dz * dz);
            I_0(1, 1) = (1.0f / 12.0f) * mass * (dx * dx + dz * dz);
            I_0(2, 2) = (1.0f / 12.0f) * mass * (dx * dx + dy * dy);

            I_0_inv = I_0.inverse();
        }
        else {
            mass_inv = 0.0f;
            I_0      = Eigen::Matrix3f::Zero();
            I_0_inv  = Eigen::Matrix3f::Zero();
        }
    }

    RigidBodyItem::RigidBodyItem(
        RigidBodyType type,
        float radius,
        float mass,
        const Eigen::Vector3f& position,
        const Eigen::Quaternionf& orientation,
        const Eigen::Vector3f& v,
        const Eigen::Vector3f& w
    ) : 
        type(RigidBodyType::Sphere),
        dim(Eigen::Vector3f::Zero()),
        radius(radius),
        mass(mass),
        position(position),
        orientation(orientation),
        v(v),
        w(w),
        force(Eigen::Vector3f::Zero()),
        torque(Eigen::Vector3f::Zero()) 
    {
        if (mass > 0.0f) {
            mass_inv = 1.0f / mass;

            float r2 = radius * radius;
            float I_sphere = (2.0f / 5.0f) * mass * r2;

            I_0 = Eigen::Matrix3f::Identity() * I_sphere;
            I_0_inv = I_0.inverse();
        }
        else {
            mass_inv = 0.0f;
            I_0      = Eigen::Matrix3f::Zero();
            I_0_inv  = Eigen::Matrix3f::Zero();
        }
    }

    void RigidBodyItem::ClearForces() {
        force.setZero();
        torque.setZero();
    }

    void RigidBodyItem::ApplyForce(const Eigen::Vector3f& f) {
        force += f; // torque remains unchanged
    }

    void RigidBodyItem::ApplyForceAtPoint(const Eigen::Vector3f& f, const Eigen::Vector3f& p) {
        force += f;
        Eigen::Vector3f r = p - position;
        torque += r.cross(f);
    }

    Eigen::Matrix3f RigidBodyItem::GetInertiaTensorWorld() const {
        Eigen::Matrix3f R = orientation.toRotationMatrix();
        return R * I_0 * R.transpose();
    }

    Eigen::Matrix3f RigidBodyItem::GetInvInertiaTensorWorld() const {
        Eigen::Matrix3f R = orientation.toRotationMatrix();
        return R * I_0_inv * R.transpose();
    }

} // namespace VCX::Labs::RigidBody