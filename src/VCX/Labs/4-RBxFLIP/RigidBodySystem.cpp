#include "RigidBodySystem.h"
#include <fcl/narrowphase/collision.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>

namespace VCX::Labs::RBxFLIP {

    void RigidBodySystem::AddBody(const RigidBodyItem& body) {
        _bodies.push_back(body);
        _sleeping.push_back(false);
        _sleepTimer.push_back(0.0f);
    }

    void RigidBodySystem::Update(float dt) {
        Integrate(dt);
        DetectCollisions();
        ResolveCollisions();
        UpdateSleeping(dt);
    }

    void RigidBodySystem::Clear() {
        _bodies.clear();
        _contacts.clear();
        _sleeping.clear();
        _sleepTimer.clear();
    }

    void RigidBodySystem::Integrate(float dt) {
        for (RigidBodyItem& body : _bodies) {
            if (body.GetMass() <= 0.0f) continue; // skip static objects

            int id = &body - &_bodies[0];
            if (body.force.squaredNorm() > 1e-6f || body.torque.squaredNorm() > 1e-6f)
                WakeBody(id);
            if (_sleeping[id]) continue; // skip sleeping objects
            if (enableGravity)
                body.ApplyForce(gravity * body.GetMass());
            // Linear motion
            Eigen::Vector3f a = body.force * body.GetInvMass();
            body.v += a * dt;
            body.position += body.v * dt;

            // Angular motion
            Eigen::Vector3f b = body.GetInvInertiaTensorWorld() * body.torque;
            body.w += b * dt;
            Eigen::Quaternionf q = body.orientation;
            Eigen::Quaternionf w_q(0, body.w.x(), body.w.y(), body.w.z());
            q.coeffs() += 0.5f * (w_q * q).coeffs() * dt;
            q.normalize();
            body.orientation = q;
            body.ClearForces();
        }
    }

    void RigidBodySystem::DetectCollisions() {
        _contacts.clear();
        int n = _bodies.size();
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j)
                collisionDetectBoxBox_fcl(i, j);
    }

    void RigidBodySystem::collisionDetectBoxBox_fcl(int id1, int id2) {
        RigidBodyItem const & b0 = _bodies[id1];
        RigidBodyItem const & b1 = _bodies[id2];
        using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<float>>;
        CollisionGeometryPtr_t geom_A;
        if (b0.GetType() == RigidBodyType::Box)
            geom_A = std::make_shared<fcl::Box<float>>(b0.GetDim()[0], b0.GetDim()[1], b0.GetDim()[2]);
        else
            geom_A = std::make_shared<fcl::Sphere<float>>(b0.radius); 
        CollisionGeometryPtr_t geom_B;
        if (b1.GetType() == RigidBodyType::Box)
            geom_B = std::make_shared<fcl::Box<float>>(b1.GetDim()[0], b1.GetDim()[1], b1.GetDim()[2]);
        else
            geom_B = std::make_shared<fcl::Sphere<float>>(b1.radius);
        fcl::CollisionObject<float> obj_A(geom_A, fcl::Transform3f(Eigen::Translation3f(b0.GetPosition()) * b0.GetOrient()));
        fcl::CollisionObject<float> obj_B(geom_B, fcl::Transform3f(Eigen::Translation3f(b1.GetPosition()) * b1.GetOrient()));
        // Compute collision - at most 8 contacts and return contact information.
        fcl::CollisionRequest<float> collisionRequest(8, true);
        fcl::CollisionResult<float> collisionResult;
        fcl::collide(&obj_A, &obj_B, collisionRequest, collisionResult);
        if(! collisionResult.isCollision()) return;
        std::vector<fcl::Contact<float>> contacts;
        collisionResult.getContacts(contacts);
        if (contacts.empty()) return;
        Eigen::Vector3f avg_pos = Eigen::Vector3f::Zero();
        Eigen::Vector3f avg_normal = Eigen::Vector3f::Zero();
        float max_depth = 0.0f;
        
        for(auto const & contact : contacts) {
            avg_pos += contact.pos;
            avg_normal += -contact.normal;
            max_depth = std::max(max_depth, contact.penetration_depth);
        }
        
        avg_pos /= contacts.size();
        avg_normal.normalize();
        
        _contacts.emplace_back(Contact(id1, id2, avg_pos, avg_normal, max_depth));
    }

    void RigidBodySystem::ResolveCollisions() {
        for (Contact& contact : _contacts) {
            RigidBodyItem& b1 = _bodies[contact.id1];
            RigidBodyItem& b2 = _bodies[contact.id2];
            Eigen::Vector3f r1 = contact.p - b1.GetPosition();
            Eigen::Vector3f r2 = contact.p - b2.GetPosition();
            Eigen::Vector3f v1 = b1.Getv() + b1.Getw().cross(r1);
            Eigen::Vector3f v2 = b2.Getv() + b2.Getw().cross(r2);
            
            if ((v1 - v2).squaredNorm() > 0.01f) {
                WakeBody(contact.id1);
                WakeBody(contact.id2);
            }
        }
        int iterations = 10;
        for (int it = 0; it < iterations; ++it)
            for (Contact& contact : _contacts)
                contact.Resolve(*this, _bodies[contact.id1], _bodies[contact.id2]);
                
        // Position solver (Pseudo-velocity / Projection)
        int positionIterations = 5;
        for (int it = 0; it < positionIterations; ++it)
            for (Contact& contact : _contacts)
                contact.ResolvePosition(*this, _bodies[contact.id1], _bodies[contact.id2]);
    }

    // enhance stability
    void RigidBodySystem::WakeBody(int id) {
        if (id < 0 || id >= (int)_sleeping.size()) return;
        _sleeping[id] = false;
        _sleepTimer[id] = 0.0f;
    }

    void RigidBodySystem::UpdateSleeping(float dt) {
    const Eigen::Vector3f up(0, 1, 0);

    std::vector<bool> hasSupport(_bodies.size(), false);
    for (const auto& c : _contacts) {
        float ndotu = std::abs(c.n.dot(up));
        if (ndotu > 0.5f) {
            hasSupport[c.id1] = true;
            hasSupport[c.id2] = true;
        }
    }

    for (int i = 0; i < (int)_bodies.size(); ++i) {
        auto& b = _bodies[i];
        if (b.GetMass() <= 0.0f) continue;

        float v2 = b.Getv().squaredNorm();
        float w2 = b.Getw().squaredNorm();

        bool lowMotion = (v2 < _sleepV && w2 < _sleepW);
        if (lowMotion && hasSupport[i]) {
            _sleepTimer[i] += dt;
            if (_sleepTimer[i] > _sleepTime) {
                _sleeping[i] = true;
                b.v.setZero();
                b.w.setZero();
            }
        } else {
            _sleepTimer[i] = 0.0f;
            _sleeping[i] = false;
        }
    }
}
} // namespace VCX::Labs::RBxFLIP