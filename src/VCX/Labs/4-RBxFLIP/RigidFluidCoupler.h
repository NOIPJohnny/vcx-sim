#pragma once

#include "FluidSimulator.h"
#include "RigidBody.h"

namespace VCX::Labs::RBxFLIP {

    class RigidFluidCoupler {
    public:
        void EnforceSphereBoundary(Simulator& fluid, RigidBodyItem const& sphere) const;
        void EnforceSphereTankBoundary(RigidBodyItem& sphere, glm::vec3 const& tankMin, glm::vec3 const& tankMax) const;
        void MarkSphereSolidCells(Simulator& fluid, RigidBodyItem const& sphere) const;
        void ApplyPressureForces(Simulator const& fluid, RigidBodyItem& sphere, float pressureScale) const;
    };

} // namespace VCX::Labs::RBxFLIP
