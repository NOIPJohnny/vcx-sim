#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "FluidSimulator.h"
#include "Labs/3-FEM/FEMSystem.h"

namespace VCX::Labs::Coupling {

    class FEMFluidCoupler {
    public:
        void ExchangeMomentum(Simulator& fluid, FEM::FEMSystem& solid, float couplingStrength);

    private:
        int CellIndex(Simulator const& fluid, glm::vec3 const& pos) const;
        void EnsureCapacity(std::size_t cellCount);

        std::vector<float> _solidMass;
        std::vector<glm::vec3> _solidMomentum;
        std::vector<glm::vec3> _mixedVelocity;
    };

} // namespace VCX::Labs::Coupling
