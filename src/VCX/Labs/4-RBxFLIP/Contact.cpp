#include "Contact.h"
#include "RigidBodySystem.h"
#include <algorithm>

namespace VCX::Labs::RBxFLIP {

    void Contact::Resolve(RigidBodySystem& system, RigidBodyItem& body1, RigidBodyItem& body2) {
        float invM1 = body1.GetMass() > 0 ? body1.GetInvMass() : 0.0f;
        float invM2 = body2.GetMass() > 0 ? body2.GetInvMass() : 0.0f;
        if (invM1 == 0.0f && invM2 == 0.0f) return;

        float restitution = system.GetRestitution(); // c
        float friction = system.GetFriction();  // μ

        Eigen::Vector3f r1 = p - body1.GetPosition();
        Eigen::Vector3f r2 = p - body2.GetPosition();

        Eigen::Vector3f v1 = body1.Getv() + body1.Getw().cross(r1);
        Eigen::Vector3f v2 = body2.Getv() + body2.Getw().cross(r2);
        Eigen::Vector3f vRel = v1 - v2;

        float vRelNormal = vRel.dot(n);
        if (vRelNormal >= 0) return;
        float actualRestitution = (vRelNormal > -0.1f) ? 0.0f : restitution;


        Eigen::Matrix3f invI1 = body1.GetMass() > 0 ? body1.GetInvInertiaTensorWorld() : Eigen::Matrix3f::Zero();
        Eigen::Matrix3f invI2 = body2.GetMass() > 0 ? body2.GetInvInertiaTensorWorld() : Eigen::Matrix3f::Zero();

        // slide 32
        float term1 = invM1 + invM2;
        float term2 = (invI1 * r1.cross(n)).cross(r1).dot(n);
        float term3 = (invI2 * r2.cross(n)).cross(r2).dot(n);
        float denominatorNormal = term1 + term2 + term3;
        float Jn = -(1.0f + actualRestitution) * vRelNormal / denominatorNormal;
        
        Eigen::Vector3f vRelTangent = vRel - vRelNormal * n;
        Eigen::Vector3f t = vRelTangent;
        float Jt = 0.0f;

        if (t.squaredNorm() > 1e-6f) {
            t.normalize();
            float denomTangent = term1 + 
                                 (invI1 * r1.cross(t)).cross(r1).dot(t) + 
                                 (invI2 * r2.cross(t)).cross(r2).dot(t);
            Jt = -vRel.dot(t) / denomTangent;
            Jt = std::clamp(Jt, -friction * Jn, friction * Jn);
        }

        Eigen::Vector3f impulse = Jn * n + Jt * t;

        if (invM1 > 0.0f) {
            body1.vShift(impulse * invM1);
            body1.wShift(invI1 * r1.cross(impulse));
        }
        if (invM2 > 0.0f) {
            body2.vShift(-impulse * invM2);
            body2.wShift(-invI2 * r2.cross(impulse));
        }
    }

    void Contact::ResolvePosition(RigidBodySystem& system, RigidBodyItem& body1, RigidBodyItem& body2) {
        float invM1 = body1.GetMass() > 0 ? body1.GetInvMass() : 0.0f;
        float invM2 = body2.GetMass() > 0 ? body2.GetInvMass() : 0.0f;
        if (invM1 == 0.0f && invM2 == 0.0f) return;

        const float percent = 0.2f;
        const float slop = 0.01f;
        
        if (Depth > slop) {
            float actual_correction = std::max(Depth - slop, 0.0f) * percent;
            Eigen::Vector3f correction = actual_correction / (invM1 + invM2) * n;
            if (invM1 > 0.0f) body1.PositionShift(invM1 * correction);
            if (invM2 > 0.0f) body2.PositionShift(-invM2 * correction);
            Depth -= actual_correction;
        }
    }
} // namespace VCX::Labs::RBxFLIP