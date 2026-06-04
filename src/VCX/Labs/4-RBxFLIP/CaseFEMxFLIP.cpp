#include <algorithm>
#include <cmath>

#include "Engine/app.h"
#include "Labs/Common/ImGuiHelper.h"
#include "CaseFEMxFLIP.h"

namespace VCX::Labs::RBxFLIP {

    inline const std::vector<glm::vec3> kBoundaryVertices = {
        {-0.5f, -0.5f, -0.5f}, { 0.5f, -0.5f, -0.5f},
        { 0.5f,  0.5f, -0.5f}, {-0.5f,  0.5f, -0.5f},
        {-0.5f, -0.5f,  0.5f}, { 0.5f, -0.5f,  0.5f},
        { 0.5f,  0.5f,  0.5f}, {-0.5f,  0.5f,  0.5f},
    };

    inline const std::vector<std::uint32_t> kBoundaryIndices = {
        0,1, 1,2, 2,3, 3,0,
        4,5, 5,6, 6,7, 7,4,
        0,4, 1,5, 2,6, 3,7
    };

    CaseFEMxFLIP::CaseFEMxFLIP() :
        _program(Engine::GL::UniqueProgram({
            Engine::GL::SharedShader("assets/shaders/fluid.vert"),
            Engine::GL::SharedShader("assets/shaders/fluid.frag"),
        })),
        _lineProgram(Engine::GL::UniqueProgram({
            Engine::GL::SharedShader("assets/shaders/flat.vert"),
            Engine::GL::SharedShader("assets/shaders/flat.frag"),
        })),
        _boundaryItem(
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Lines
        ),
        _softEdgeItem(
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Lines
        ),
        _sceneObject(1),
        _fluidSphere(Engine::Model{ Engine::Sphere(6, 0.02f), 0 }),
        _softSphere(Engine::Model{ Engine::Sphere(8, 0.03f), 0 }) {

        _program.BindUniformBlock("PassConstants", 1);
        _program.GetUniforms().SetByName("u_AmbientScale", 1.0f);
        _program.GetUniforms().SetByName("u_UseBlinn", 1);
        _program.GetUniforms().SetByName("u_Shininess", 32.0f);
        _program.GetUniforms().SetByName("u_UseGammaCorrection", 1);
        _program.GetUniforms().SetByName("u_AttenuationOrder", 2);

        _lineProgram.GetUniforms().SetByName("u_Color", glm::vec3(1.0f));
        _boundaryItem.UpdateElementBuffer(kBoundaryIndices);

        _sceneObject.Camera.Eye = glm::vec3(0.9f, 0.9f, 1.3f);
        _sceneObject.Camera.Target = glm::vec3(0.0f);

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_sceneObject.Camera);

        _sim.m_fRatio = 0.95f;
        ResetSystem();
    }

    void CaseFEMxFLIP::ResetSystem() {
        _sim.m_fRatio = std::clamp(_sim.m_fRatio, 0.0f, 1.0f);
        _sim.setupScene(_gridResolution, _initialFluidSize);
        _fluidSphere = Engine::Model{ Engine::Sphere(6, _sim.m_particleRadius), 0 };

        _softSystem.wx = static_cast<std::size_t>(_softGridX);
        _softSystem.wy = static_cast<std::size_t>(_softGridY);
        _softSystem.wz = static_cast<std::size_t>(_softGridZ);
        _softSystem.delta = _softSpacing;
        _softSystem.E = 1000.0f;
        _softSystem.nu = 0.25f;
        _softSystem.density = 400.0f;
        _softSystem.gravity = glm::vec3(0.0f, -9.81f, 0.0f);
        _softSystem.damping = 2.0f;
        _softSystem.enableCollision = false;
        _softSystem.useSphereCollider = false;
        _softSystem.model = std::make_unique<FEM::CorotatedModel>();
        _softSystem.integrator = std::make_unique<FEM::ExplicitIntegrator>();
        _softSystem.ResetSystem();

        std::fill(_softSystem.fixed.begin(), _softSystem.fixed.end(), false);
        CenterSoftBody(_softCenter);

        for (auto & v : _softSystem.velocities)
            v = glm::vec3(0.0f);
        for (auto & f : _softSystem.externalForces)
            f = glm::vec3(0.0f);

        _softSphere = Engine::Model{ Engine::Sphere(8, _softSpacing * 0.35f), 0 };
        UpdateSoftEdgeIndices();
        UpdateSoftEdgeVertices();
    }

    void CaseFEMxFLIP::OnSetupPropsUI() {
        if (ImGui::Button("Reset"))
            ResetSystem();
        ImGui::SameLine();
        if (ImGui::Button(_paused ? "Start" : "Pause"))
            _paused = !_paused;

        ImGui::Spacing();
        ImGui::Checkbox("Use Fixed dt", &_useFixedDt);
        ImGui::SliderFloat("Fixed dt", &_fixedDt, 1.0f / 240.0f, 1.0f / 80.0f, "%.5f");
        ImGui::SliderFloat("Time Scale", &_timeScale, 0.1f, 2.0f, "%.2f");

        ImGui::SeparatorText("Fluid");
        ImGui::SliderInt("Grid Resolution", &_gridResolution, 20, 40);
        ImGui::SliderInt("Sub Steps", &_numSubSteps, 1, 4);
        ImGui::SliderInt("Particle Iters", &_numParticleIters, 1, 10);
        ImGui::SliderInt("Pressure Iters", &_numPressureIters, 5, 80);
        ImGui::SliderFloat("Over Relaxation", &_overRelaxation, 0.1f, 1.9f, "%.2f");
        ImGui::Checkbox("Compensate Drift", &_compensateDrift);
        ImGui::Checkbox("Separate Particles", &_separateParticles);
        ImGui::SliderFloat("Initial Fluid Size X", &_initialFluidSize.x, 0.1f, 0.95f, "%.2f");
        ImGui::SliderFloat("Initial Fluid Size Y", &_initialFluidSize.y, 0.1f, 0.95f, "%.2f");
        ImGui::SliderFloat("Initial Fluid Size Z", &_initialFluidSize.z, 0.1f, 0.95f, "%.2f");
        ImGui::SliderFloat("flipRatio", &_sim.m_fRatio, 0.0f, 1.0f, "%.2f");

        if (ImGui::Button("Apply Grid Reset"))
            ResetSystem();

        ImGui::SeparatorText("Soft Body");
        ImGui::SliderInt("Soft Grid X", &_softGridX, 2, 8);
        ImGui::SliderInt("Soft Grid Y", &_softGridY, 2, 8);
        ImGui::SliderInt("Soft Grid Z", &_softGridZ, 2, 8);
        ImGui::SliderFloat("Soft Spacing", &_softSpacing, 0.04f, 0.1f, "%.3f");
        ImGui::SliderFloat("Soft Center Y", &_softCenter.y, -0.1f, 0.4f, "%.2f");
        ImGui::SliderFloat("Young's Modulus", &_softSystem.E, 1000.0f, 80000.0f, "%.0f");
        ImGui::SliderFloat("Poisson Ratio", &_softSystem.nu, 0.1f, 0.45f, "%.2f");
        ImGui::SliderFloat("Density", &_softSystem.density, 200.0f, 1000.0f, "%.0f");
        ImGui::SliderFloat("Damping", &_softSystem.damping, 0.0f, 6.0f, "%.2f");
        ImGui::SliderInt("Soft Substeps", &_softSubsteps, 1, 20);

        if (ImGui::Button("Apply Soft Reset"))
            ResetSystem();

        ImGui::SeparatorText("Coupling");
        ImGui::SliderFloat("Coupling Strength", &_couplingStrength, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Coupling Damping", &_couplingDamping, 0.0f, 6.0f, "%.2f");
    }

    void CaseFEMxFLIP::SolvePressure(float dt) {
        _sim.solveIncompressibility(_numPressureIters, dt, _overRelaxation, _compensateDrift);
    }

    void CaseFEMxFLIP::StepSimulation(float dt) {
        float const sdt = dt / static_cast<float>(_numSubSteps);

        for (int step = 0; step < _numSubSteps; ++step) {
            _sim.integrateParticles(sdt);
            _sim.handleParticleCollisions(glm::vec3(0.0f), 0.0f, glm::vec3(0.0f));

            if (_separateParticles)
                _sim.pushParticlesApart(_numParticleIters);
            _sim.handleParticleCollisions(glm::vec3(0.0f), 0.0f, glm::vec3(0.0f));

            _sim.transferVelocities(true, _sim.m_fRatio);
            _sim.updateParticleDensity();

            SolvePressure(sdt);
            _coupler.ExchangeMomentum(_sim, _softSystem, _couplingStrength);
            _sim.transferVelocities(false, _sim.m_fRatio);

            int const softSteps = std::max(_softSubsteps, 1);
            float const softDt = sdt / static_cast<float>(softSteps);
            for (int i = 0; i < softSteps; ++i)
                _softSystem.Update(softDt);
            ApplySoftDamping(sdt);
            ConstrainSoftBody();
        }

        _sim.updateParticleColors();
        UpdateSoftEdgeVertices();
    }

    void CaseFEMxFLIP::ApplySoftDamping(float dt) {
        float const factor = std::max(0.0f, 1.0f - _couplingDamping * dt);
        for (auto & v : _softSystem.velocities)
            v *= factor;
    }

    void CaseFEMxFLIP::ConstrainSoftBody() {
        float const margin = std::max(_sim.m_h * 2.0f, 0.02f);
        float const minCoord = -0.5f + margin;
        float const maxCoord = 0.5f - margin;

        for (std::size_t i = 0; i < _softSystem.positions.size(); ++i) {
            auto & pos = _softSystem.positions[i];
            auto & vel = _softSystem.velocities[i];
            for (int axis = 0; axis < 3; ++axis) {
                if (pos[axis] < minCoord) {
                    pos[axis] = minCoord;
                    if (vel[axis] < 0.0f) vel[axis] = 0.0f;
                }
                if (pos[axis] > maxCoord) {
                    pos[axis] = maxCoord;
                    if (vel[axis] > 0.0f) vel[axis] = 0.0f;
                }
            }
        }
    }

    void CaseFEMxFLIP::CenterSoftBody(glm::vec3 const & center) {
        if (_softSystem.positions.empty())
            return;

        glm::vec3 minPos = _softSystem.positions.front();
        glm::vec3 maxPos = _softSystem.positions.front();
        for (auto const & pos : _softSystem.positions) {
            minPos = glm::min(minPos, pos);
            maxPos = glm::max(maxPos, pos);
        }
        glm::vec3 currentCenter = 0.5f * (minPos + maxPos);
        glm::vec3 const shift = center - currentCenter;
        for (auto & pos : _softSystem.positions)
            pos += shift;
    }

    void CaseFEMxFLIP::UpdateSoftEdgeIndices() {
        _softEdgeIndices.resize(_softSystem.tets.size() * 12);
        for (std::size_t i = 0; i < _softEdgeIndices.size(); ++i)
            _softEdgeIndices[i] = static_cast<std::uint32_t>(i);
        _softEdgeItem.UpdateElementBuffer(_softEdgeIndices);
    }

    void CaseFEMxFLIP::UpdateSoftEdgeVertices() {
        _softEdgeVertices.clear();
        _softEdgeVertices.reserve(_softEdgeIndices.size());

        for (auto const & tet : _softSystem.tets) {
            int const ids[4] = { tet.indices[0], tet.indices[1], tet.indices[2], tet.indices[3] };
            int const edges[6][2] = {
                {0, 1}, {0, 2}, {0, 3},
                {1, 2}, {1, 3}, {2, 3},
            };
            for (auto const & edge : edges) {
                _softEdgeVertices.push_back(_softSystem.positions[ids[edge[0]]]);
                _softEdgeVertices.push_back(_softSystem.positions[ids[edge[1]]]);
            }
        }

        _softEdgeItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_softEdgeVertices));
    }

    Common::CaseRenderResult CaseFEMxFLIP::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        float dt = _useFixedDt ? _fixedDt : Engine::GetDeltaTime() * _timeScale;
        dt = std::clamp(dt, 1.0f / 300.0f, 1.0f / 80.0f);

        if (!_paused)
            StepSimulation(dt);
        else {
            _sim.updateParticleColors();
            UpdateSoftEdgeVertices();
        }

        _frame.Resize(desiredSize);
        _boundaryItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(kBoundaryVertices));

        _cameraManager.Update(_sceneObject.Camera);

        Rendering::SceneObject::PassConstants pass {};
        pass.Projection = _sceneObject.Camera.GetProjectionMatrix(float(desiredSize.first) / float(desiredSize.second));
        pass.View = _sceneObject.Camera.GetViewMatrix();
        pass.ViewPosition = _sceneObject.Camera.Eye;
        pass.AmbientIntensity = glm::vec3(0.6f);
        pass.CntPointLights = 0;
        pass.CntSpotLights = 0;
        pass.CntDirectionalLights = 0;
        _sceneObject.PassConstantsBlock.Update(pass);

        _lineProgram.GetUniforms().SetByName("u_Projection", pass.Projection);
        _lineProgram.GetUniforms().SetByName("u_View", pass.View);

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glLineWidth(2.0f);
        _lineProgram.GetUniforms().SetByName("u_Color", glm::vec3(1.0f));
        _boundaryItem.Draw({ _lineProgram.Use() });
        glLineWidth(1.0f);

        Rendering::ModelObject fluidParticles(_fluidSphere, _sim.m_particlePos, _sim.m_particleColor);
        fluidParticles.Mesh.Draw({ _program.Use() }, _fluidSphere.Mesh.Indices.size(), 0, _sim.m_particlePos.size());

        _lineProgram.GetUniforms().SetByName("u_Color", glm::vec3(0.85f, 0.35f, 0.2f));
        _softEdgeItem.Draw({ _lineProgram.Use() });

        _softColors.assign(_softSystem.positions.size(), glm::vec3(0.9f, 0.6f, 0.35f));
        Rendering::ModelObject softParticles(_softSphere, _softSystem.positions, _softColors);
        softParticles.Mesh.Draw({ _program.Use() }, _softSphere.Mesh.Indices.size(), 0, _softSystem.positions.size());

        glDisable(GL_DEPTH_TEST);

        return Common::CaseRenderResult{
            .Fixed = false,
            .Flipped = true,
            .Image = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseFEMxFLIP::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_sceneObject.Camera, pos);
    }

} // namespace VCX::Labs::RBxFLIP
