#include "FEMFluidCoupler.h"

#include <algorithm>
#include <cmath>

namespace VCX::Labs::Coupling {

    int FEMFluidCoupler::CellIndex(Simulator const& fluid, glm::vec3 const& pos) const {
        glm::vec3 const gridMin(-0.5f);
        glm::vec3 const rel = (pos - gridMin) * fluid.m_fInvSpacing;
        int const i = std::clamp(static_cast<int>(std::floor(rel.x)), 0, fluid.m_iCellX - 1);
        int const j = std::clamp(static_cast<int>(std::floor(rel.y)), 0, fluid.m_iCellY - 1);
        int const k = std::clamp(static_cast<int>(std::floor(rel.z)), 0, fluid.m_iCellZ - 1);
        return i * (fluid.m_iCellY * fluid.m_iCellZ) + j * fluid.m_iCellZ + k;
    }

    void FEMFluidCoupler::EnsureCapacity(std::size_t cellCount) {
        _solidMass.assign(cellCount, 0.0f);
        _solidMomentum.assign(cellCount, glm::vec3(0.0f));
        _mixedVelocity.assign(cellCount, glm::vec3(0.0f));
    }

    void FEMFluidCoupler::ExchangeMomentum(Simulator& fluid, FEM::FEMSystem& solid, float couplingStrength) {
        if (fluid.m_iNumCells == 0 || solid.positions.empty())
            return;

        couplingStrength = std::clamp(couplingStrength, 0.0f, 1.0f);
        EnsureCapacity(static_cast<std::size_t>(fluid.m_iNumCells));

        for (std::size_t i = 0; i < solid.positions.size(); ++i) {
            float const mass = solid.masses[i];
            if (mass <= 0.0f)
                continue;
            int const cell = CellIndex(fluid, solid.positions[i]);
            _solidMass[cell] += mass;
            _solidMomentum[cell] += mass * solid.velocities[i];
        }

        for (std::size_t cell = 0; cell < _solidMass.size(); ++cell) {
            float const solidMass = _solidMass[cell];
            if (solidMass <= 0.0f)
                continue;

            glm::vec3 const solidVel = _solidMomentum[cell] / solidMass;
            float const fluidMass = fluid.m_particleDensity[cell];
            float const totalMass = fluidMass + solidMass;
            if (totalMass <= 0.0f)
                continue;

            glm::vec3 const mixed = (fluidMass * fluid.m_vel[cell] + solidMass * solidVel) / totalMass;
            _mixedVelocity[cell] = mixed;
            fluid.m_vel[cell] = glm::mix(fluid.m_vel[cell], mixed, couplingStrength);
        }

        for (std::size_t i = 0; i < solid.positions.size(); ++i) {
            float const mass = solid.masses[i];
            if (mass <= 0.0f)
                continue;
            int const cell = CellIndex(fluid, solid.positions[i]);
            if (_solidMass[cell] <= 0.0f)
                continue;

            glm::vec3 const target = _mixedVelocity[cell];
            solid.velocities[i] = glm::mix(solid.velocities[i], target, couplingStrength);
        }
    }

    void FEMFluidCoupler::ExchangeMomentum(Simulator& fluid, FEM::TetMesh& solid, float couplingStrength) {
        if (fluid.m_iNumCells == 0 || solid.NumVertices() == 0)
            return;

        couplingStrength = std::clamp(couplingStrength, 0.0f, 1.0f);
        EnsureCapacity(static_cast<std::size_t>(fluid.m_iNumCells));

        for (int i = 0; i < solid.NumVertices(); ++i) {
            float const mass = solid.masses[i];
            if (mass <= 0.0f)
                continue;
            glm::vec3 const pos(solid.positions[i].x(), solid.positions[i].y(), solid.positions[i].z());
            glm::vec3 const vel(solid.velocities[i].x(), solid.velocities[i].y(), solid.velocities[i].z());
            int const cell = CellIndex(fluid, pos);
            _solidMass[cell] += mass;
            _solidMomentum[cell] += mass * vel;
        }

        for (std::size_t cell = 0; cell < _solidMass.size(); ++cell) {
            float const solidMass = _solidMass[cell];
            if (solidMass <= 0.0f)
                continue;

            glm::vec3 const solidVel = _solidMomentum[cell] / solidMass;
            float const fluidMass = fluid.m_particleDensity[cell];
            float const totalMass = fluidMass + solidMass;
            if (totalMass <= 0.0f)
                continue;

            glm::vec3 const mixed = (fluidMass * fluid.m_vel[cell] + solidMass * solidVel) / totalMass;
            _mixedVelocity[cell] = mixed;
            fluid.m_vel[cell] = glm::mix(fluid.m_vel[cell], mixed, couplingStrength);
        }

        for (int i = 0; i < solid.NumVertices(); ++i) {
            float const mass = solid.masses[i];
            if (mass <= 0.0f)
                continue;
            glm::vec3 const pos(solid.positions[i].x(), solid.positions[i].y(), solid.positions[i].z());
            int const cell = CellIndex(fluid, pos);
            if (_solidMass[cell] <= 0.0f)
                continue;

            Eigen::Vector3f const & curVel = solid.velocities[i];
            glm::vec3 const curVelG(curVel.x(), curVel.y(), curVel.z());
            glm::vec3 const mixed = glm::mix(curVelG, _mixedVelocity[cell], couplingStrength);
            solid.velocities[i] = Eigen::Vector3f(mixed.x, mixed.y, mixed.z);
        }
    }

} // namespace VCX::Labs::Coupling
