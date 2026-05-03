#pragma once

#include <cstdint>
#include <utility>
#include <vector>
#include <glm/glm.hpp>

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Engine/Sphere.h"
#include "Engine/Scene.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Scene/SceneObject.h"

#include "FEMSystem.h"

namespace VCX::Labs::FEM {

    class CaseFEM : public Common::ICase {
    public:
        CaseFEM();

        std::string_view const GetName() override { return "FEM Simulation"; }
        void OnSetupPropsUI() override;
        Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        void OnProcessInput(ImVec2 const & pos) override;

    private:
        enum class ElasticModel : int {
            Linear = 0,
            StVK = 1,
            NeoHookean = 2,
            Corotated = 3,
        };

        enum class IntegratorMode : int {
            Explicit = 0,
            Implicit = 1,
        };

    private:
        void ResetSystem();
        void ApplyModel();
        void ApplyIntegrator();
        void StepSimulation(float dt);
        void ApplyMouseForce(ImVec2 const & mousePos);
        void UpdateRenderData();
        void UpdateTetEdgeIndices();

    private:
        FEMSystem _system;

        Engine::GL::UniqueProgram _program;
        Engine::GL::UniqueProgram _lineProgram;
        Engine::GL::UniqueRenderFrame _frame;
        Engine::GL::UniqueIndexedRenderItem _particleItem;
        Engine::GL::UniqueIndexedRenderItem _edgeItem;
        Engine::Model _particleModel;

        Engine::Camera _camera { .Eye = glm::vec3(5.5f, 4.0f, 9.5f), .Target = glm::vec3(4.0f, 1.0f, 1.0f) };
        Rendering::SceneObject _sceneObject;
        Common::OrbitCameraManager _cameraManager;

        bool _paused = true;
        bool _stepOnce = false;
        float _fixedDt = 1.0f / 600.0f;
        int _substeps = 4;

        ElasticModel _elasticModel = ElasticModel::Linear;
        IntegratorMode _integratorMode = IntegratorMode::Explicit;
        int _newtonIters = 8;
        float _newtonTolerance = 1e-4f;

        ImVec2 _lastMousePos = ImVec2(0.0f, 0.0f);
        bool _hasLastMousePos = false;
        std::pair<std::uint32_t, std::uint32_t> _viewportSize { 1, 1 };
        int _controlledVertex = -1;
        float _mouseForceScale = 5000.0f;
        float _mouseMaxForce = 200000.0f;

        std::vector<glm::vec3> _particleOffsets;
        std::vector<glm::vec3> _particleColors;
        std::vector<glm::vec3> _edgeVertices;
        std::vector<std::uint32_t> _edgeIndices;

        float _lastStepDt = 1.0f / 600.0f;
    };

} // namespace VCX::Labs::FEM
