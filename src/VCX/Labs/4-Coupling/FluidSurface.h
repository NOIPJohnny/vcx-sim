#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "FluidSimulator.h"

namespace VCX::Labs::Coupling {

    struct FluidSurfaceMesh {
        std::vector<glm::vec3>     positions;
        std::vector<glm::vec3>     normals;
        std::vector<std::uint32_t> indices;
    };

    FluidSurfaceMesh BuildFluidSurface(Simulator const & sim, float radiusScale = 2.0f);

} // namespace VCX::Labs::Coupling
