#pragma once
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "RigidBody.h"
#include "Contact.h"

namespace VCX::Labs::Coupling {

    class RigidBodySystem {
    public:
        RigidBodySystem() = default;

        void AddBody(const RigidBodyItem& body);
        void Update(float dt);
        const std::vector<RigidBodyItem>& GetBodies() const { return _bodies; }
        std::vector<RigidBodyItem>& GetBodies() { return _bodies; }
        const std::vector<Contact>& GetContacts() const { return _contacts; }
        const float GetRestitution() const { return _restitution; }
        const float GetFriction() const { return _friction; }
        void SetRestitution(float r) { _restitution = r; }
        void SetFriction(float f) { _friction = f; }
        void Clear();
        void EnableGravity() { enableGravity = true; }
        void DisableGravity() { enableGravity = false; }
        void SetLinearDamping(float d)  { _linearDamping = d; }
        void SetAngularDamping(float d) { _angularDamping = d; }

    private:
        std::vector<RigidBodyItem> _bodies;
        std::vector<Contact> _contacts;
        Eigen::Vector3f gravity = Eigen::Vector3f(0, -9.8f, 0);
        bool enableGravity = false;
        float _restitution = 0.3f; // c
        float _friction = 0.3f;  // μ
        float _linearDamping  = 1.0f;   // drag coefficient k: v *= exp(-k * dt)
        float _angularDamping = 1.0f;   // drag coefficient k: w *= exp(-k * dt)

        void DetectCollisions();
        void ResolveCollisions();
        void Integrate(float dt);
        void collisionDetectBoxBox_fcl(int id1, int id2);

        // enhance stability
        std::vector<bool> _sleeping;
        std::vector<float> _sleepTimer;
        float _sleepTime = 0.6f; // time to sleep
        float _sleepV = 0.05f; // linear velocity threshold
        float _sleepW = 0.05f; // angular velocity threshold
        void WakeBody(int id);
        void UpdateSleeping(float dt);
    };
} // namespace VCX::Labs::Coupling