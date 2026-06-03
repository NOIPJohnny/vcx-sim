#pragma once

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Engine/Sphere.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Scene/SceneObject.h"
#include "FluidSimulator.h"
#include "RigidBodySystem.h"
#include "RigidFluidCoupler.h"

namespace VCX::Labs::RBxFLIP {

    class CaseRBxFLIP : public Common::ICase {
    public:
        CaseRBxFLIP();

        std::string_view const GetName() override { return "RBxFLIP"; }
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
            APIC = 1,
        };

    private:
        void ResetSystem();
        void StepFlip(float dt);
        void StepAPIC(float dt);
        void SolvePressure(float dt);
        void UpdateColorByMode();
        void UpdateObstacleFromInput(ImVec2 const & pos);
        int ParticleToCellOffset(glm::vec3 const & p) const;
        RigidBodyItem* SphereBody();
        RigidBodyItem const* SphereBody() const;
        void StopSphereMotion();
        void DrawRigidSphere(glm::mat4 const& projection, glm::mat4 const& view);

    private:
        Engine::GL::UniqueProgram _program;
        Engine::GL::UniqueProgram _lineProgram;
        Engine::GL::UniqueProgram _rigidProgram;
        Engine::GL::UniqueRenderFrame _frame;
        Engine::GL::UniqueIndexedRenderItem _boundaryItem;
        Engine::GL::UniqueIndexedRenderItem _rigidSphereItem;

        Rendering::SceneObject _sceneObject;
        Common::OrbitCameraManager _cameraManager;
        Engine::Model _sphere;

        Simulator _sim;
        RigidBodySystem _rigidBodySystem;
        RigidFluidCoupler _coupler;

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
        glm::vec3 _initialFluidSize = glm::vec3(0.6f, 0.8f, 0.9f);

        ColorMode _colorMode = ColorMode::Speed;
        PressureMode _pressureMode = PressureMode::GaussSeidel;
        SimMode _simMode = SimMode::FLIP;

        bool _enableFluidToRigid = true;
        bool _enableRigidToFluid = true;
        float _pressureScale = 0.15f;
        bool _fixSphere = false;

        bool _enableObstacle = true;
        float _obstacleRadius = 0.08f;
        glm::vec3 _obstaclePos = glm::vec3(0.0f);
        glm::vec3 _obstacleVel = glm::vec3(0.0f);
        float _obstacleDragScale = 0.0025f;

        ImVec2 _lastMousePos = ImVec2(0.0f, 0.0f);
        float _lastStepDt = 1.0f / 60.0f;
    };

} // namespace VCX::Labs::RBxFLIP
