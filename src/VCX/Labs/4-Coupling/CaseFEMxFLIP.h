#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Engine/Sphere.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Scene/SceneObject.h"
#include "FluidSimulator.h"
#include "FEMFluidCoupler.h"
#include "Labs/3-FEM/FEMSystem.h"
#include "Labs/3-FEM/HyperElasticModels.h"
#include "Labs/3-FEM/Integrator.h"

namespace VCX::Labs::Coupling {

    class CaseFEMxFLIP : public Common::ICase {
    public:
        CaseFEMxFLIP();

        std::string_view const GetName() override { return "FEMxFLIP"; }
        void OnSetupPropsUI() override;
        Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        void OnProcessInput(ImVec2 const & pos) override;

    private:
        void ResetSystem();
        void StepSimulation(float dt);
        void SolvePressure(float dt);
        void UpdateSoftEdgeIndices();
        void UpdateSoftEdgeVertices();
        void ConstrainSoftBody();
        void ApplySoftDamping(float dt);
        void CenterSoftBody(glm::vec3 const & center);

    private:
        Engine::GL::UniqueProgram _program;
        Engine::GL::UniqueProgram _lineProgram;
        Engine::GL::UniqueRenderFrame _frame;
        Engine::GL::UniqueIndexedRenderItem _boundaryItem;
        Engine::GL::UniqueIndexedRenderItem _softEdgeItem;

        Rendering::SceneObject _sceneObject;
        Common::OrbitCameraManager _cameraManager;
        Engine::Model _fluidSphere;
        Engine::Model _softSphere;

        Simulator _sim;
        FEM::FEMSystem _softSystem;
        FEMFluidCoupler _coupler;

        bool _paused = false;
        bool _useFixedDt = true;
        float _fixedDt = 1.0f / 60.0f;
        float _timeScale = 1.0f;

        int _gridResolution = 24;
        int _numSubSteps = 1;
        int _numParticleIters = 5;
        int _numPressureIters = 30;
        float _overRelaxation = 0.5f;
        bool _compensateDrift = true;
        bool _separateParticles = true;
        glm::vec3 _initialFluidSize = glm::vec3(0.6f, 0.8f, 0.9f);

        int _softGridX = 4;
        int _softGridY = 4;
        int _softGridZ = 4;
        float _softSpacing = 0.06f;
        glm::vec3 _softCenter = glm::vec3(0.0f, 0.25f, 0.0f);
        int _softSubsteps = 5;

        float _couplingStrength = 0.6f;
        float _couplingDamping = 1.5f;

        std::vector<glm::vec3> _softEdgeVertices;
        std::vector<std::uint32_t> _softEdgeIndices;
        std::vector<glm::vec3> _softColors;
    };

} // namespace VCX::Labs::Coupling
