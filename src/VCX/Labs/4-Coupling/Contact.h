#pragma once
#include <Eigen/Core>
#include "RigidBody.h"

namespace VCX::Labs::Coupling {

    struct Contact {
        int id1,id2;
        Eigen::Vector3f p; // point of contact
        Eigen::Vector3f n; // normal
        float Depth; // penetration depth

        Contact(int id1, int id2, const Eigen::Vector3f& p, const Eigen::Vector3f& n, float depth)
            : id1(id1), id2(id2), p(p), n(n), Depth(depth) {}

        void Resolve(RigidBodySystem& system, RigidBodyItem& body1, RigidBodyItem& body2);
        void ResolvePosition(RigidBodySystem& system, RigidBodyItem& body1, RigidBodyItem& body2);
    };

} // namespace VCX::Labs::Coupling