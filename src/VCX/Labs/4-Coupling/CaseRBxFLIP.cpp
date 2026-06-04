#include <algorithm>
#include <cmath>

#include "Engine/app.h"
#include "Labs/Common/ImGuiHelper.h"
#include "CaseRBxFLIP.h"

namespace VCX::Labs::Coupling {

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

    static glm::vec3 MixColor(glm::vec3 const & a, glm::vec3 const & b, float t) {
        t = std::clamp(t, 0.0f, 1.0f);
        return a * (1.0f - t) + b * t;
    }

    CaseRBxFLIP::CaseRBxFLIP() :
        _program(Engine::GL::UniqueProgram({
            Engine::GL::SharedShader("assets/shaders/fluid.vert"),
            Engine::GL::SharedShader("assets/shaders/fluid.frag"),
        })),
        _lineProgram(Engine::GL::UniqueProgram({
            Engine::GL::SharedShader("assets/shaders/flat.vert"),
            Engine::GL::SharedShader("assets/shaders/flat.frag"),
        })),
        _rigidProgram(Engine::GL::UniqueProgram({
            Engine::GL::SharedShader("assets/shaders/flat.vert"),
            Engine::GL::SharedShader("assets/shaders/flat.frag"),
        })),
        _boundaryItem(
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Lines
        ),
        _rigidSphereItem(
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Triangles
        ),
        _sceneObject(1),
        _sphere(Engine::Model{ Engine::Sphere(6, 0.02f), 0 }) {

        _program.BindUniformBlock("PassConstants", 1);
        _program.GetUniforms().SetByName("u_AmbientScale", 1.0f);
        _program.GetUniforms().SetByName("u_UseBlinn", 1);
        _program.GetUniforms().SetByName("u_Shininess", 32.0f);
        _program.GetUniforms().SetByName("u_UseGammaCorrection", 1);
        _program.GetUniforms().SetByName("u_AttenuationOrder", 2);

        _lineProgram.GetUniforms().SetByName("u_Color", glm::vec3(1.0f));
        _boundaryItem.UpdateElementBuffer(kBoundaryIndices);
        _rigidProgram.GetUniforms().SetByName("u_Color", glm::vec3(0.85f, 0.68f, 0.28f));
        _rigidSphereItem.UpdateElementBuffer(Engine::Sphere(20, 1.0f).GetIndices());

        _sceneObject.Camera.Eye = glm::vec3(0.9f, 0.9f, 1.3f);
        _sceneObject.Camera.Target = glm::vec3(0.0f);

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_sceneObject.Camera);

        _sim.m_fRatio = 0.95f;
        ResetSystem();
    }

    void CaseRBxFLIP::ResetSystem() {
        _sim.m_fRatio = std::clamp(_sim.m_fRatio, 0.0f, 1.0f);
        _sim.setupScene(_gridResolution, _initialFluidSize);

        _sphere = Engine::Model{ Engine::Sphere(6, _sim.m_particleRadius), 0 };
        _obstaclePos = glm::vec3(0.0f);
        _obstacleVel = glm::vec3(0.0f);

        _rigidBodySystem.Clear();
        _rigidBodySystem.EnableGravity();
        _rigidBodySystem.AddBody(RigidBodyItem(
            RigidBodyType::Sphere,
            0.08f,
            0.004f,
            Eigen::Vector3f(0.0f, 0.40f, 0.0f)
        ));
    }

    void CaseRBxFLIP::OnSetupPropsUI() {
        if (ImGui::Button("Reset")) {
            ResetSystem();
        }
        ImGui::SameLine();
        if (ImGui::Button(_paused ? "Start" : "Pause")) {
            _paused = !_paused;
        }

        ImGui::Spacing();

        ImGui::Checkbox("Use Fixed dt", &_useFixedDt);
        ImGui::SliderFloat("Fixed dt", &_fixedDt, 1.0f / 240.0f, 1.0f / 100.0f, "%.5f");
        ImGui::SliderFloat("Time Scale", &_timeScale, 0.1f, 2.0f, "%.2f");
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
        ImGui::Checkbox("Enable Fluid->Rigid", &_enableFluidToRigid);
        ImGui::Checkbox("Enable Rigid->Fluid", &_enableRigidToFluid);
        ImGui::SliderFloat("Pressure Scale", &_pressureScale, 0.0f, 2.0f, "%.3f");
        ImGui::Checkbox("Fix Sphere", &_fixSphere);

        if (ImGui::Button("Apply Grid Reset")) {
            ResetSystem();
        }

        ImGui::SeparatorText("B1");
        {
            int cm = static_cast<int>(_colorMode);
            ImGui::RadioButton("Color: Speed", &cm, static_cast<int>(ColorMode::Speed));
            ImGui::SameLine();
            ImGui::RadioButton("Density", &cm, static_cast<int>(ColorMode::Density));
            ImGui::SameLine();
            ImGui::RadioButton("Pressure", &cm, static_cast<int>(ColorMode::Pressure));
            _colorMode = static_cast<ColorMode>(cm);
        }

        ImGui::SeparatorText("B2");
        {
            int pm = static_cast<int>(_pressureMode);
            ImGui::RadioButton("GS", &pm, static_cast<int>(PressureMode::GaussSeidel));
            ImGui::SameLine();
            ImGui::RadioButton("CG", &pm, static_cast<int>(PressureMode::CG));
            ImGui::SameLine();
            ImGui::RadioButton("PCG", &pm, static_cast<int>(PressureMode::PCG));
            _pressureMode = static_cast<PressureMode>(pm);
        }

        ImGui::SeparatorText("B4");
        {
            int sm = static_cast<int>(_simMode);
            ImGui::RadioButton("FLIP", &sm, static_cast<int>(SimMode::FLIP));
            ImGui::SameLine();
            ImGui::RadioButton("APIC", &sm, static_cast<int>(SimMode::APIC));
            _simMode = static_cast<SimMode>(sm);

            if (_simMode == SimMode::FLIP)
                ImGui::SliderFloat("flipRatio", &_sim.m_fRatio, 0.0f, 1.0f, "%.2f");
            else
                ImGui::TextUnformatted("APIC has no flip ratio.");
        }


    }

    void CaseRBxFLIP::SolvePressure(float dt) {
        if (_pressureMode == PressureMode::GaussSeidel)
            _sim.solveIncompressibility(_numPressureIters, dt, _overRelaxation, _compensateDrift);
    
        else if (_pressureMode == PressureMode::CG)
            _sim.solveIncompressibilityCG(dt, _compensateDrift, false);
    
        
        else if (_pressureMode == PressureMode::PCG)
            _sim.solveIncompressibilityCG(dt, _compensateDrift, true);
    }

    void CaseRBxFLIP::UpdateColorByMode() {
        if (_colorMode == ColorMode::Speed) {
            _sim.updateParticleColors();
            return;
        }

        float const rest = std::max(_sim.m_particleRestDensity, 1e-6f);
        float pMean = 1.0f;

        if (_colorMode == ColorMode::Pressure) {
            float sum = 0.0f;
            int count = 0;

            for (int idx = 0; idx < _sim.m_iNumCells; ++idx) {
                if (_sim.m_type[idx] != FLUID_CELL)
                    continue;

                float const p = _sim.m_p[idx];
                if (!std::isfinite(p))
                    continue;

                sum += std::abs(p);
                ++count;
            }

            if (count > 0) {
                pMean = sum / static_cast<float>(count);
            }
        }

        for (std::size_t i = 0; i < _sim.m_particlePos.size(); ++i) {
            int const cell = ParticleToCellOffset(_sim.m_particlePos[i]);

            if (_colorMode == ColorMode::Density) {
                float const rho = _sim.m_particleDensity[cell] / rest;
                _sim.m_particleColor[i] = MixColor(
                    glm::vec3(0.2f, 0.4f, 1.0f),
                    glm::vec3(1.0f, 0.3f, 0.2f),
                    rho * 0.75f
                );
            } else {
                float const p = _sim.m_p[cell];

                if (!std::isfinite(p)) {
                    _sim.m_particleColor[i] = glm::vec3(1.0f, 1.0f, 0.0f);
                    continue;
                }

                float t = std::abs(p) / std::max(2.0f * pMean, 1e-6f);
                t = std::clamp(t, 0.0f, 1.0f);

                _sim.m_particleColor[i] = MixColor(
                    glm::vec3(0.2f, 1.0f, 0.6f),
                    glm::vec3(1.0f, 0.2f, 0.7f),
                    t
                );
            }
        }
    }



    int CaseRBxFLIP::ParticleToCellOffset(glm::vec3 const & p) const {
        glm::vec3 const gp = (p + glm::vec3(0.5f)) * _sim.m_fInvSpacing;
        int const i = std::clamp(static_cast<int>(gp.x), 0, _sim.m_iCellX - 1);
        int const j = std::clamp(static_cast<int>(gp.y), 0, _sim.m_iCellY - 1);
        int const k = std::clamp(static_cast<int>(gp.z), 0, _sim.m_iCellZ - 1);
        return i * (_sim.m_iCellY * _sim.m_iCellZ) + j * _sim.m_iCellZ + k;
    }

    RigidBodyItem* CaseRBxFLIP::SphereBody() {
        auto& bodies = _rigidBodySystem.GetBodies();
        return bodies.empty() ? nullptr : &bodies[0];
    }

    RigidBodyItem const* CaseRBxFLIP::SphereBody() const {
        auto const& bodies = _rigidBodySystem.GetBodies();
        return bodies.empty() ? nullptr : &bodies[0];
    }

    void CaseRBxFLIP::StopSphereMotion() {
        RigidBodyItem* sphere = SphereBody();
        if (sphere == nullptr)
            return;

        sphere->vShift(-sphere->Getv());
        sphere->wShift(-sphere->Getw());
        sphere->ClearForces();
    }

    void CaseRBxFLIP::StepFlip(float dt) {
        float const sdt = dt / static_cast<float>(_numSubSteps);

        for (int step = 0; step < _numSubSteps; ++step) {
            RigidBodyItem* sphere = SphereBody();
            _sim.integrateParticles(sdt);
            if (_enableRigidToFluid && sphere != nullptr)
                _coupler.EnforceSphereBoundary(_sim, *sphere);

            _sim.handleParticleCollisions(glm::vec3(0.0f), 0.0f, glm::vec3(0.0f));

            if (_separateParticles) {
                _sim.pushParticlesApart(_numParticleIters);
            }

            if (_enableRigidToFluid && sphere != nullptr)
                _coupler.EnforceSphereBoundary(_sim, *sphere);

            _sim.transferVelocities(true, _sim.m_fRatio);
            if (_enableRigidToFluid && sphere != nullptr)
                _coupler.MarkSphereSolidCells(_sim, *sphere);
            _sim.updateParticleDensity();
            SolvePressure(sdt);
            _sim.transferVelocities(false, _sim.m_fRatio);

            if (sphere != nullptr) {
                if (_fixSphere) {
                    StopSphereMotion();
                } else {
                    if (_enableFluidToRigid)
                        _coupler.ApplyPressureForces(_sim, *sphere, _pressureScale);
                    _rigidBodySystem.Update(sdt);
                    _coupler.EnforceSphereTankBoundary(*sphere, glm::vec3(-0.5f), glm::vec3(0.5f));
                }
            }
        }

        UpdateColorByMode();
    }

    void CaseRBxFLIP::StepAPIC(float dt) {
        float const sdt = dt / static_cast<float>(_numSubSteps);

        for (int step = 0; step < _numSubSteps; ++step) {
            RigidBodyItem* sphere = SphereBody();
            _sim.integrateParticles(sdt);
            if (_enableRigidToFluid && sphere != nullptr)
                _coupler.EnforceSphereBoundary(_sim, *sphere);

            _sim.handleParticleCollisions(glm::vec3(0.0f), 0.0f, glm::vec3(0.0f));

            if (_separateParticles) {
                _sim.pushParticlesApart(_numParticleIters);
            }

            if (_enableRigidToFluid && sphere != nullptr)
                _coupler.EnforceSphereBoundary(_sim, *sphere);

            _sim.transferVelocitiesAPIC(true);
            if (_enableRigidToFluid && sphere != nullptr)
                _coupler.MarkSphereSolidCells(_sim, *sphere);
            _sim.updateParticleDensity();
            SolvePressure(sdt);
            _sim.transferVelocitiesAPIC(false);

            if (sphere != nullptr) {
                if (_fixSphere) {
                    StopSphereMotion();
                } else {
                    if (_enableFluidToRigid)
                        _coupler.ApplyPressureForces(_sim, *sphere, _pressureScale);
                    _rigidBodySystem.Update(sdt);
                    _coupler.EnforceSphereTankBoundary(*sphere, glm::vec3(-0.5f), glm::vec3(0.5f));
                }
            }
        }

        UpdateColorByMode();
    }

    void CaseRBxFLIP::DrawRigidSphere(glm::mat4 const& projection, glm::mat4 const& view) {
        RigidBodyItem const* body = SphereBody();
        if (body == nullptr)
            return;

        static Engine::Sphere sphereTemplate(20, 1.0f);
        static auto sphereLocalVerts = sphereTemplate.GetVertices();

        Eigen::Vector3f const center = body->GetPosition();
        Eigen::Quaternionf const q = body->GetOrient();
        float const radius = body->GetRadius();

        std::vector<glm::vec3> verts(sphereLocalVerts.size());
        for (std::size_t i = 0; i < sphereLocalVerts.size(); ++i) {
            Eigen::Vector3f local(sphereLocalVerts[i].x, sphereLocalVerts[i].y, sphereLocalVerts[i].z);
            Eigen::Vector3f world = center + q * (local * radius);
            verts[i] = glm::vec3(world.x(), world.y(), world.z());
        }

        _rigidProgram.GetUniforms().SetByName("u_Projection", projection);
        _rigidProgram.GetUniforms().SetByName("u_View", view);
        _rigidProgram.GetUniforms().SetByName("u_Color", _fixSphere ? glm::vec3(0.9f, 0.85f, 0.35f) : glm::vec3(0.85f, 0.62f, 0.28f));
        _rigidSphereItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(verts));
        _rigidSphereItem.Draw({ _rigidProgram.Use() });
    }

    Common::CaseRenderResult CaseRBxFLIP::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        float dt = _useFixedDt ? _fixedDt : Engine::GetDeltaTime() * _timeScale;
        dt = std::clamp(dt, 1.0f / 300.0f, 1.0f / 100.0f);
        _lastStepDt = dt;

        if (!_paused) {
            if (_simMode == SimMode::FLIP)
                StepFlip(dt);
            else
                StepAPIC(dt);
        }
        else UpdateColorByMode();
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
        glBindBufferBase(GL_UNIFORM_BUFFER, 1, _sceneObject.PassConstantsBlock.Get());

        _lineProgram.GetUniforms().SetByName("u_Projection", pass.Projection);
        _lineProgram.GetUniforms().SetByName("u_View", pass.View);

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glLineWidth(2.0f);
        _boundaryItem.Draw({_lineProgram.Use()});
        glLineWidth(1.0f);

        Rendering::ModelObject particles(_sphere, _sim.m_particlePos, _sim.m_particleColor);
        particles.Mesh.Draw({_program.Use()}, _sphere.Mesh.Indices.size(), 0, _sim.m_particlePos.size());

        DrawRigidSphere(pass.Projection, pass.View);
        glDisable(GL_DEPTH_TEST);

        return Common::CaseRenderResult{
            .Fixed = false,
            .Flipped = true,
            .Image = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseRBxFLIP::UpdateObstacleFromInput(ImVec2 const & pos) {
        if (!_enableObstacle) {
            _obstacleVel = glm::vec3(0.0f);
            _lastMousePos = pos;
            return;
        }

        auto const & io = ImGui::GetIO();
        bool const dragging = io.KeyAlt && ImGui::IsMouseDown(ImGuiMouseButton_Left);

        if (!dragging) {
            _obstacleVel = glm::vec3(0.0f);
            _lastMousePos = pos;
            return;
        }

        ImVec2 const d = ImVec2(pos.x - _lastMousePos.x, pos.y - _lastMousePos.y);
        _lastMousePos = pos;

        glm::vec3 const deltaWorld(d.x * _obstacleDragScale, -d.y * _obstacleDragScale, 0.0f);
        _obstaclePos += deltaWorld;

        float const lim = 0.5f - _obstacleRadius;
        _obstaclePos.x = std::clamp(_obstaclePos.x, -lim, lim);
        _obstaclePos.y = std::clamp(_obstaclePos.y, -lim, lim);
        _obstaclePos.z = std::clamp(_obstaclePos.z, -lim, lim);

        _obstacleVel = deltaWorld / std::max(_lastStepDt, 1e-6f);
    }

    void CaseRBxFLIP::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_sceneObject.Camera, pos);
    }

} // namespace VCX::Labs::Coupling
