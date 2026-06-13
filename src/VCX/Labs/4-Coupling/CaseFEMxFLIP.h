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
#include "FluidSurface.h"
#include "FEMFluidCoupler.h"
#include "TetMesh.h"
#include "FEMIntegrator.h"
#include "Labs/3-FEM/FEMSoftBodyBuilder.h"

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
        void RebuildSoftMesh();
        void StepSimulation(float dt);
        void SolvePressure(float dt);
        void ConstrainSoftBody();
        void CenterSoftBody(glm::vec3 const & center);
        void DrawFluidSurface(glm::mat4 const & projection, glm::mat4 const & view);

    private:
        // fluid rendering
        Engine::GL::UniqueProgram _program;
        Engine::GL::UniqueProgram _lineProgram;
        Engine::GL::UniqueProgram _litProgram;
        Engine::GL::UniqueProgram _fluidSurfaceProgram;
        Engine::GL::UniqueRenderFrame _frame;
        Engine::GL::UniqueIndexedRenderItem _boundaryItem;
        Engine::GL::UniqueIndexedRenderItem _softEdgeItem;
        Engine::GL::UniqueIndexedRenderItem _surfaceItem;
        Engine::GL::UniqueIndexedRenderItem _fluidSurfaceItem;

        Rendering::SceneObject _sceneObject;
        Common::OrbitCameraManager _cameraManager;
        Engine::Model _fluidSphere;
        Engine::Model _softSphere;

        Simulator _sim;
        FEM::TetMesh _softMesh;
        FEM::FEMIntegrator _femIntegrator;
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

        int _softGridX = 5;
        int _softGridY = 16;
        int _softGridZ = 5;
        float _softSpacing = 0.03f;
        glm::vec3 _softCenter = glm::vec3(-0.02f, -0.06f, -0.02f);
        FEM::SoftBodyType _softBodyType = FEM::SoftBodyType::GridBlock;
        float _totalMass = 0.03f;

        FEM::MaterialModel _materialModel = FEM::MaterialModel::StVK;
        float _lambda = 300.0f;
        float _mu = 50.0f;
        float _damping = 4.5f;

        int _softSubsteps = 50;

        float _couplingStrength = 0.6f;

        // surface appearance
        glm::vec3 _surfaceColor = glm::vec3(0.9f, 0.55f, 0.2f);
        bool _showWireframe = true;
        bool _showSurface = true;
        bool _showVertices = true;
        bool _showFluidSurface = true;
        bool _showFluidParticles = false;
        glm::vec3 _fluidSurfaceColor = glm::vec3(0.16f, 0.52f, 0.95f);
        float _fluidSurfaceAlpha = 0.62f;
        bool _useLighting = true;
        float _lightIntensity = 1.0f;
        float _ambientScale = 0.30f;
        float _shininess = 150.0f;
        bool _flatShading = true;
        glm::vec3 _lightDir = glm::vec3(7.0f, 9.0f, 7.0f);

        std::vector<glm::vec3> _softEdgeVertices;
        std::vector<std::uint32_t> _softEdgeIndices;
        std::vector<glm::vec3> _softColors;
        std::vector<std::uint32_t> _surfaceTriIndices;
    };

} // namespace VCX::Labs::Coupling
