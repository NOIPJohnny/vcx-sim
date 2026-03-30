#include "RigidBodySystem.h"
#include <fcl/narrowphase/collision.h>

namespace VCX::Labs::RigidBody {

    void RigidBodySystem::AddBody(const RigidBodyItem& body) {
        _bodies.push_back(body);
    }

    void RigidBodySystem::Update(float dt) {
        Integrate(dt);
        DetectCollisions();
        ResolveCollisions();
    }

    void RigidBodySystem::Clear() {
        _bodies.clear();
        _contacts.clear();
    }

    void RigidBodySystem::Integrate(float dt) {
        for (RigidBodyItem& body : _bodies) {
            if (body.GetMass() <= 0.0f) continue; // skip static objects
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
        //modified the original code to adapt to my own RigidBodyItem
        RigidBodyItem const & b0 = _bodies[id1];
        RigidBodyItem const & b1 = _bodies[id2];
        // Eigen::Vector3f RigidBody::dim - size of a box
        using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<float>>;
        CollisionGeometryPtr_t box_geometry_A(new fcl::Box<float>(b0.dim[0], b0.dim[1], b0.dim[2]));
        CollisionGeometryPtr_t box_geometry_B(new fcl::Box<float>(b1.dim[0], b1.dim[1], b1.dim[2]));
        // Eigen::Vector3f RigidBody::x - position of a box, Eigen::Quaternionf RigidBody::q - rotation of a box
        fcl::CollisionObject<float> box_A(box_geometry_A, fcl::Transform3f(Eigen::Translation3f(b0.position)*b0.orientation));
        fcl::CollisionObject<float> box_B(box_geometry_B, fcl::Transform3f(Eigen::Translation3f(b1.position)*b1.orientation));
        // Compute collision - at most 8 contacts and return contact information.
        fcl::CollisionRequest<float> collisionRequest(8, true);
        fcl::CollisionResult<float> collisionResult;
        fcl::collide(&box_A, &box_B, collisionRequest, collisionResult);
        if(! collisionResult.isCollision()) return;
        std::vector<fcl::Contact<float>> contacts;
        collisionResult.getContacts(contacts);
        // You can decide whether define your own Contact
        for(auto const & contact : contacts) {
            _contacts.emplace_back(Contact(id1, id2, contact.pos, -contact.normal, contact.penetration_depth));
        }
    }

    void RigidBodySystem::ResolveCollisions() {
        for (Contact& contact : _contacts)
            contact.Resolve(_bodies[contact.id1], _bodies[contact.id2]);
    }
} // namespace VCX::Labs::RigidBody