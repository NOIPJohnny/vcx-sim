#pragma once

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/Sphere.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Scene/SceneObject.h"
#include "FluidSimulator.h"

namespace VCX::Labs::Fluid {

    class CaseFluid : public Common::ICase {
    public:
        CaseFluid();

        std::string_view const GetName() override { return "Fluid Simulation"; }
        void OnSetupPropsUI() override;
        Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        void OnProcessInput(ImVec2 const & pos) override;

    private:
        enum class ColorMode : int {
            Speed = 0,
            Density = 1,
            Pressure = 2,
        };

        enum class PressureMode : int {
            GaussSeidel = 0,
            CG = 1,
            PCG = 2,
        };

        enum class SimMode : int {
            FLIP = 0,
            Eulerian = 1,
        };

    private:
        void ResetSystem();
        void StepFlip(float dt);
        void SolvePressure(float dt);
        void UpdateColorByMode();
        void UpdateObstacleFromInput(ImVec2 const & pos);
        int ParticleToCellOffset(glm::vec3 const & p) const;

    private:
        Engine::GL::UniqueProgram _program;
        Engine::GL::UniqueProgram _lineProgram;
        Engine::GL::UniqueRenderFrame _frame;
        Engine::GL::UniqueIndexedRenderItem _boundaryItem;

        Rendering::SceneObject _sceneObject;
        Common::OrbitCameraManager _cameraManager;
        Engine::Model _sphere;

        Fluid::Simulator _sim;

        bool _paused = false;
        bool _useFixedDt = true;
        float _fixedDt = 1.0f / 60.0f;
        float _timeScale = 1.0f;

        int _gridResolution = 28;
        int _numSubSteps = 1;
        int _numParticleIters = 5;
        int _numPressureIters = 30;
        float _overRelaxation = 0.5f;
        bool _compensateDrift = true;
        bool _separateParticles = true;

        ColorMode _colorMode = ColorMode::Speed;
        PressureMode _pressureMode = PressureMode::GaussSeidel;
        SimMode _simMode = SimMode::FLIP;

        bool _enableObstacle = true;
        float _obstacleRadius = 0.08f;
        glm::vec3 _obstaclePos = glm::vec3(0.0f);
        glm::vec3 _obstacleVel = glm::vec3(0.0f);
        float _obstacleDragScale = 0.0025f;

        ImVec2 _lastMousePos = ImVec2(0.0f, 0.0f);
        float _lastStepDt = 1.0f / 60.0f;
    };

} // namespace VCX::Labs::FluidSimulation