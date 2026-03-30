#pragma once
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "RigidBody.h"
#include "Contact.h"

namespace VCX::Labs::RigidBody {

    class RigidBodySystem {
    public:
        RigidBodySystem() = default;

        void AddBody(const RigidBodyItem& body);
        void Update(float dt);
        const std::vector<RigidBodyItem>& GetBodies() const { return _bodies; }
        std::vector<RigidBodyItem>& GetBodies() { return _bodies; }
        const std::vector<Contact>& GetContacts() const { return _contacts; }
        void Clear();
        void EnableGravity() { enableGravity = true; }
        void DisableGravity() { enableGravity = false; }

    private:
        std::vector<RigidBodyItem> _bodies;
        std::vector<Contact> _contacts;
        Eigen::Vector3f gravity = Eigen::Vector3f(0, -9.8f, 0);
        bool enableGravity = false;

        void DetectCollisions();
        void ResolveCollisions();
        void Integrate(float dt);
        void collisionDetectBoxBox_fcl(int id1, int id2);
    };
} // namespace VCX::Labs::RigidBody