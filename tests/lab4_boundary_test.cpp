#include <cmath>

#include <glm/glm.hpp>

#include "Labs/4-RBxFLIP/RigidBody.h"
#include "Labs/4-RBxFLIP/RigidBodySystem.h"
#include "Labs/4-RBxFLIP/RigidFluidCoupler.h"

int main() {
    using namespace VCX::Labs::RBxFLIP;

    RigidFluidCoupler coupler;
    RigidBodyItem sphere(
        RigidBodyType::Sphere,
        0.08f,
        0.05f,
        Eigen::Vector3f(0.0f, -0.60f, 0.0f),
        Eigen::Quaternionf::Identity(),
        Eigen::Vector3f(0.0f, -1.0f, 0.0f)
    );

    coupler.EnforceSphereTankBoundary(sphere, glm::vec3(-0.5f), glm::vec3(0.5f));

    if (std::abs(sphere.GetPosition().y() - (-0.42f)) >= 1e-5f)
        return 1;
    if (sphere.Getv().y() != 0.0f)
        return 1;

    Simulator fluid;
    fluid.m_iCellX = 5;
    fluid.m_iCellY = 5;
    fluid.m_iCellZ = 5;
    fluid.m_h = 0.2f;
    fluid.m_fInvSpacing = 5.0f;
    fluid.m_iNumCells = fluid.m_iCellX * fluid.m_iCellY * fluid.m_iCellZ;
    fluid.m_p.assign(fluid.m_iNumCells, 0.0f);
    fluid.m_s.assign(fluid.m_iNumCells, 1.0f);
    fluid.m_type.assign(fluid.m_iNumCells, EMPTY_CELL);

    int const solid = fluid.index2GridOffset(glm::ivec3(2, 2, 2));
    int const fluidBelow = fluid.index2GridOffset(glm::ivec3(2, 1, 2));
    fluid.m_s[solid] = 0.0f;
    fluid.m_type[solid] = SOLID_CELL;
    fluid.m_type[fluidBelow] = FLUID_CELL;
    fluid.m_p[fluidBelow] = 10.0f;

    RigidBodySystem system;
    system.AddBody(RigidBodyItem(
        RigidBodyType::Sphere,
        0.11f,
        0.03f,
        Eigen::Vector3f(0.0f, 0.0f, 0.0f)
    ));

    auto& pressureSphere = system.GetBodies()[0];
    coupler.ApplyPressureForces(fluid, pressureSphere, 1.0f);
    system.Update(1.0f / 60.0f);

    if (pressureSphere.Getv().y() <= 0.0f)
        return 1;
    return 0;
}
