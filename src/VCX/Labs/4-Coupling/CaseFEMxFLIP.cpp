#include <algorithm>
#include <cmath>
#include <set>

#include <glm/gtc/type_ptr.hpp>

#include "Engine/app.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Labs/3-FEM/FEMSystem.h"
#include "CaseFEMxFLIP.h"

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

    CaseFEMxFLIP::CaseFEMxFLIP() :
        _program(Engine::GL::UniqueProgram({
            Engine::GL::SharedShader("assets/shaders/fluid.vert"),
            Engine::GL::SharedShader("assets/shaders/fluid.frag"),
        })),
        _lineProgram(Engine::GL::UniqueProgram({
            Engine::GL::SharedShader("assets/shaders/flat.vert"),
            Engine::GL::SharedShader("assets/shaders/flat.frag"),
        })),
        _litProgram(Engine::GL::UniqueProgram({
            Engine::GL::SharedShader("assets/shaders/lit_mesh.vert"),
            Engine::GL::SharedShader("assets/shaders/lit_mesh.frag"),
        })),
        _fluidSurfaceProgram(Engine::GL::UniqueProgram({
            Engine::GL::SharedShader("assets/shaders/lit_mesh.vert"),
            Engine::GL::SharedShader("assets/shaders/fluid_surface.frag"),
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
        _surfaceItem(
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0)
                .Add<glm::vec3>("normal",   Engine::GL::DrawFrequency::Stream, 1),
            Engine::GL::PrimitiveType::Triangles
        ),
        _fluidSurfaceItem(
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0)
                .Add<glm::vec3>("normal",   Engine::GL::DrawFrequency::Stream, 1),
            Engine::GL::PrimitiveType::Triangles
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
        RebuildSoftMesh();
    }

    void CaseFEMxFLIP::RebuildSoftMesh() {
        _femIntegrator.material.lambda = _lambda;
        _femIntegrator.material.mu     = _mu;
        _femIntegrator.materialModel   = _materialModel;

        FEM::FEMSystem structure;
        structure.wx = static_cast<std::size_t>(std::max(_softGridX, 1));
        structure.wy = static_cast<std::size_t>(std::max(_softGridY, 1));
        structure.wz = static_cast<std::size_t>(std::max(_softGridZ, 1));
        structure.delta = _softSpacing;
        structure.softBodyType = _softBodyType;
        FEM::BuildSoftBodyStructure(structure, _softBodyType);

        _softMesh.restPositions.clear();
        _softMesh.restPositions.reserve(structure.positions.size());
        for (auto const & p : structure.positions)
            _softMesh.restPositions.emplace_back(p.x, p.y, p.z);

        if (! _softMesh.restPositions.empty()) {
            Eigen::Vector3f minPos = _softMesh.restPositions[0];
            Eigen::Vector3f maxPos = _softMesh.restPositions[0];
            for (auto const & p : _softMesh.restPositions) {
                minPos = minPos.cwiseMin(p);
                maxPos = maxPos.cwiseMax(p);
            }
            Eigen::Vector3f const currentCenter = 0.5f * (minPos + maxPos);
            Eigen::Vector3f const targetCenter(_softCenter.x, _softCenter.y, _softCenter.z);
            Eigen::Vector3f const offset = targetCenter - currentCenter;
            for (auto & p : _softMesh.restPositions)
                p += offset;
        }

        _softMesh.positions = _softMesh.restPositions;
        _softMesh.velocities.assign(_softMesh.restPositions.size(), Eigen::Vector3f::Zero());
        _softMesh.masses.assign(_softMesh.restPositions.size(),
            _softMesh.restPositions.empty() ? 0.0f : _totalMass / static_cast<float>(_softMesh.restPositions.size()));
        _softMesh.fixed.assign(_softMesh.restPositions.size(), false);

        _softMesh.tets.clear();
        _softMesh.tets.reserve(structure.tets.size());
        for (auto const & tet : structure.tets) {
            _softMesh.tets.emplace_back(
                tet.indices[0],
                tet.indices[1],
                tet.indices[2],
                tet.indices[3]);
        }

        _softMesh.DmInv.resize(_softMesh.tets.size());
        _softMesh.restVolume.resize(_softMesh.tets.size());
        for (std::size_t i = 0; i < _softMesh.tets.size(); ++i) {
            auto const & tv = _softMesh.tets[i];
            Eigen::Vector3f const & x0 = _softMesh.restPositions[tv[0]];
            Eigen::Vector3f const & x1 = _softMesh.restPositions[tv[1]];
            Eigen::Vector3f const & x2 = _softMesh.restPositions[tv[2]];
            Eigen::Vector3f const & x3 = _softMesh.restPositions[tv[3]];

            Eigen::Matrix3f dm;
            dm.col(0) = x1 - x0;
            dm.col(1) = x2 - x0;
            dm.col(2) = x3 - x0;
            _softMesh.DmInv[i] = dm.inverse();
            _softMesh.restVolume[i] = std::abs(dm.determinant()) / 6.0f;
        }

        _softMesh.ExtractSurfaceFaces();

        _softSphere = Engine::Model{ Engine::Sphere(8, _softSpacing * 0.35f), 0 };

        // build surface triangle index buffer
        {
            _surfaceTriIndices.clear();
            _surfaceTriIndices.reserve(_softMesh.surfaceFaces.size() * 3);
            for (auto const & f : _softMesh.surfaceFaces) {
                _surfaceTriIndices.push_back(f[0]);
                _surfaceTriIndices.push_back(f[1]);
                _surfaceTriIndices.push_back(f[2]);
            }
            _surfaceItem.UpdateElementBuffer(_surfaceTriIndices);
        }

        // build wireframe line index buffer (deduplicated edges)
        {
            std::set<std::pair<int, int>> edgeSet;
            for (auto const & f : _softMesh.surfaceFaces) {
                for (int e = 0; e < 3; ++e) {
                    int a = f[e], b = f[(e + 1) % 3];
                    if (a > b) std::swap(a, b);
                    edgeSet.insert({ a, b });
                }
            }
            _softEdgeIndices.clear();
            _softEdgeIndices.reserve(edgeSet.size() * 2);
            for (auto const & [a, b] : edgeSet) {
                _softEdgeIndices.push_back(a);
                _softEdgeIndices.push_back(b);
            }
            _softEdgeItem.UpdateElementBuffer(_softEdgeIndices);
        }
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

        if (ImGui::CollapsingHeader("Fluid", ImGuiTreeNodeFlags_DefaultOpen)) {
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
        }

        if (ImGui::CollapsingHeader("Soft Body", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::BeginCombo("Soft Body Type", FEM::SoftBodyTypeName(_softBodyType))) {
                for (int i = 0; i < FEM::SoftBodyTypeCount(); ++i) {
                    auto const type = static_cast<FEM::SoftBodyType>(i);
                    bool const selected = type == _softBodyType;
                    if (ImGui::Selectable(FEM::SoftBodyTypeName(type), selected)) {
                        _softBodyType = type;
                        if (type == FEM::SoftBodyType::TeddyBear) {
                            _softGridX = std::max(_softGridX, 12);
                            _softGridY = std::max(_softGridY, 12);
                            _softGridZ = std::max(_softGridZ, 12);
                        }
                    }
                    if (selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }

            {
                int currentModel = static_cast<int>(_materialModel);
                if (ImGui::Combo("Model", &currentModel, "StVK\0Neo-Hookean\0Corotated\0"))
                    _materialModel = static_cast<FEM::MaterialModel>(currentModel);
            }
            ImGui::SliderFloat("Lambda", &_lambda, 100.0f, 1000.0f, "%.0f");
            ImGui::SliderFloat("Mu", &_mu, 10.0f, 200.0f, "%.0f");
            ImGui::SliderFloat("Damping", &_damping, 0.0f, 8.0f, "%.2f");
            ImGui::SliderInt("Soft Substeps", &_softSubsteps, 1, 200);

            ImGui::Spacing();
            ImGui::SliderInt("Soft Grid X", &_softGridX, 2, 16);
            ImGui::SliderInt("Soft Grid Y", &_softGridY, 2, 16);
            ImGui::SliderInt("Soft Grid Z", &_softGridZ, 2, 16);
            ImGui::SliderFloat("Soft Spacing", &_softSpacing, 0.02f, 0.1f, "%.3f");
            ImGui::SliderFloat("Soft Center X", &_softCenter.x, -0.4f, 0.4f, "%.2f");
            ImGui::SliderFloat("Soft Center Y", &_softCenter.y, -0.4f, 0.4f, "%.2f");
            ImGui::SliderFloat("Soft Center Z", &_softCenter.z, -0.4f, 0.4f, "%.2f");
            ImGui::SliderFloat("Total Mass", &_totalMass, 0.1f, 5.0f, "%.2f");

            if (ImGui::Button("Apply Soft Reset"))
                ResetSystem();
        }

        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Show Fluid Surface", &_showFluidSurface);
            ImGui::SameLine();
            ImGui::Checkbox("Show Fluid Particles", &_showFluidParticles);
            ImGui::ColorEdit3("Fluid Surface Color", glm::value_ptr(_fluidSurfaceColor));
            ImGui::SliderFloat("Fluid Surface Alpha", &_fluidSurfaceAlpha, 0.15f, 1.0f, "%.2f");
            ImGui::Separator();
            ImGui::Checkbox("Show Surface", &_showSurface);
            ImGui::SameLine();
            ImGui::Checkbox("Show Wireframe", &_showWireframe);
            ImGui::SameLine();
            ImGui::Checkbox("Show Vertices", &_showVertices);
            ImGui::ColorEdit3("Surface Color", glm::value_ptr(_surfaceColor));
            ImGui::Checkbox("Lighting", &_useLighting);
            if (_useLighting) {
                ImGui::SliderFloat("Light Intensity", &_lightIntensity, 0.1f, 3.0f);
                ImGui::SliderFloat("Ambient", &_ambientScale, 0.0f, 0.5f);
                ImGui::SliderFloat("Shininess", &_shininess, 1.0f, 256.0f);
                ImGui::Checkbox("Flat Shading", &_flatShading);
            }
        }

        if (ImGui::CollapsingHeader("Coupling", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("Coupling Strength", &_couplingStrength, 0.0f, 1.0f, "%.2f");
        }
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
            _coupler.ExchangeMomentum(_sim, _softMesh, _couplingStrength);
            _sim.transferVelocities(false, _sim.m_fRatio);

            Eigen::Vector3f const gravity(0.0f, -9.81f, 0.0f);
            float const g = glm::length(glm::vec3(0.0f, 9.81f, 0.0f));

            // estimate water surface height (average across all columns)
            float sumSurfaceY = 0.0f;
            int numColumns = 0;
            for (int xi = 0; xi < _sim.m_iCellX; ++xi) {
                for (int zi = 0; zi < _sim.m_iCellZ; ++zi) {
                    for (int yj = _sim.m_iCellY - 1; yj >= 0; --yj) {
                        int idx = _sim.index2GridOffset(glm::ivec3(xi, yj, zi));
                        if (_sim.m_type[idx] == FLUID_CELL) {
                            sumSurfaceY += -0.5f + (yj + 0.5f) * _sim.m_h;
                            ++numColumns;
                            break;
                        }
                    }
                }
            }
            float const avgSurfaceY = numColumns > 0 ? sumSurfaceY / numColumns : 0.0f;

            float totalVolume = 0.0f;
            for (float volume : _softMesh.restVolume)
                totalVolume += volume;
            float const volPerVertex = _softMesh.NumVertices() > 0 ? totalVolume / _softMesh.NumVertices() : 0.0f;
            float const buoyancyScale = 0.5f;

            int const softSteps = std::max(_softSubsteps, 1);
            float const softDt = sdt / static_cast<float>(softSteps);

            for (int i = 0; i < softSteps; ++i) {
                std::vector<Eigen::Vector3f> forces;
                _femIntegrator.ComputeAllForces(_softMesh, forces);

                for (int v = 0; v < _softMesh.NumVertices(); ++v) {
                    if (_softMesh.fixed[v])
                        continue;

                    forces[v] += _softMesh.masses[v] * gravity;

                    // buoyancy: Archimedes per vertex
                    if (_sim.m_particleRestDensity > 0.0f && _softMesh.positions[v].y() < avgSurfaceY) {
                        forces[v].y() += buoyancyScale * _sim.m_particleRestDensity * volPerVertex * g;
                    }

                    _softMesh.velocities[v] += softDt * forces[v] / _softMesh.masses[v];

                    float const dampFactor = std::exp(-_damping * softDt);
                    _softMesh.velocities[v] *= dampFactor;

                    _softMesh.positions[v] += softDt * _softMesh.velocities[v];

                    if (!_softMesh.positions[v].allFinite() || !_softMesh.velocities[v].allFinite()) {
                        _softMesh.positions[v] = _softMesh.restPositions[v];
                        _softMesh.velocities[v] = Eigen::Vector3f::Zero();
                    }
                }
            }

            ConstrainSoftBody();
        }

        _sim.updateParticleColors();
    }

    void CaseFEMxFLIP::ConstrainSoftBody() {
        float const margin = std::max(_sim.m_h * 2.0f, 0.03f);
        float const minCoord = -0.5f + margin;
        float const maxCoord = 0.5f - margin;

        for (int i = 0; i < _softMesh.NumVertices(); ++i) {
            auto & pos = _softMesh.positions[i];
            auto & vel = _softMesh.velocities[i];
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
        if (_softMesh.NumVertices() == 0)
            return;

        Eigen::Vector3f minPos = _softMesh.positions[0];
        Eigen::Vector3f maxPos = _softMesh.positions[0];
        for (int i = 0; i < _softMesh.NumVertices(); ++i) {
            minPos = minPos.cwiseMin(_softMesh.positions[i]);
            maxPos = maxPos.cwiseMax(_softMesh.positions[i]);
        }
        Eigen::Vector3f currentCenter = 0.5f * (minPos + maxPos);
        Eigen::Vector3f const shift(center.x, center.y, center.z);
        Eigen::Vector3f const offset = shift - currentCenter;
        for (int i = 0; i < _softMesh.NumVertices(); ++i) {
            _softMesh.positions[i] += offset;
            if (i < static_cast<int>(_softMesh.restPositions.size()))
                _softMesh.restPositions[i] += offset;
        }
    }

    void CaseFEMxFLIP::DrawFluidSurface(glm::mat4 const & projection, glm::mat4 const & view) {
        if (! _showFluidSurface)
            return;

        FluidSurfaceMesh surface = BuildFluidSurface(_sim);
        if (surface.indices.empty() ||
            surface.positions.empty() ||
            surface.positions.size() != surface.normals.size()) {
            return;
        }

        _fluidSurfaceProgram.GetUniforms().SetByName("u_Projection", projection);
        _fluidSurfaceProgram.GetUniforms().SetByName("u_View", view);
        _fluidSurfaceProgram.GetUniforms().SetByName("u_ViewPosition", _sceneObject.Camera.Eye);
        _fluidSurfaceProgram.GetUniforms().SetByName("u_Color", _fluidSurfaceColor);
        _fluidSurfaceProgram.GetUniforms().SetByName("u_Alpha", _fluidSurfaceAlpha);
        _fluidSurfaceProgram.GetUniforms().SetByName("u_LightDir", glm::normalize(glm::vec3(0.4f, 0.8f, 0.5f)));
        _fluidSurfaceProgram.GetUniforms().SetByName("u_LightColor", glm::vec3(1.35f));
        _fluidSurfaceProgram.GetUniforms().SetByName("u_AmbientColor", glm::vec3(0.28f));
        _fluidSurfaceProgram.GetUniforms().SetByName("u_Shininess", 90.0f);

        _fluidSurfaceItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(surface.positions));
        _fluidSurfaceItem.UpdateVertexBuffer("normal", Engine::make_span_bytes<glm::vec3>(surface.normals));
        _fluidSurfaceItem.UpdateElementBuffer(surface.indices);

        GLboolean depthMask = GL_TRUE;
        glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
        bool const blendWasEnabled = glIsEnabled(GL_BLEND) == GL_TRUE;
        bool const cullWasEnabled = glIsEnabled(GL_CULL_FACE) == GL_TRUE;

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        glDisable(GL_CULL_FACE);
        _fluidSurfaceItem.Draw({ _fluidSurfaceProgram.Use() });
        glDepthMask(depthMask);
        if (! blendWasEnabled) glDisable(GL_BLEND);
        if (cullWasEnabled) glEnable(GL_CULL_FACE);
    }

    Common::CaseRenderResult CaseFEMxFLIP::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        float dt = _useFixedDt ? _fixedDt : Engine::GetDeltaTime() * _timeScale;
        dt = std::clamp(dt, 1.0f / 300.0f, 1.0f / 80.0f);

        if (!_paused)
            StepSimulation(dt);
        else
            _sim.updateParticleColors();

        _frame.Resize(desiredSize);
        _boundaryItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(kBoundaryVertices));

        _cameraManager.Update(_sceneObject.Camera);

        auto const projMat = _sceneObject.Camera.GetProjectionMatrix(float(desiredSize.first) / float(desiredSize.second));
        auto const viewMat = _sceneObject.Camera.GetViewMatrix();

        Rendering::SceneObject::PassConstants pass {};
        pass.Projection = projMat;
        pass.View = viewMat;
        pass.ViewPosition = _sceneObject.Camera.Eye;
        pass.AmbientIntensity = glm::vec3(0.6f);
        pass.CntPointLights = 0;
        pass.CntSpotLights = 0;
        pass.CntDirectionalLights = 0;
        _sceneObject.PassConstantsBlock.Update(pass);
        glBindBufferBase(GL_UNIFORM_BUFFER, 1, _sceneObject.PassConstantsBlock.Get());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // tank boundary
        glLineWidth(2.0f);
        _lineProgram.GetUniforms().SetByName("u_Color", glm::vec3(1.0f));
        _lineProgram.GetUniforms().SetByName("u_Projection", projMat);
        _lineProgram.GetUniforms().SetByName("u_View", viewMat);
        _boundaryItem.Draw({ _lineProgram.Use() });
        glLineWidth(1.0f);

        // fluid particles
        if (_showFluidParticles) {
            Rendering::ModelObject fluidParticles(_fluidSphere, _sim.m_particlePos, _sim.m_particleColor);
            fluidParticles.Mesh.Draw({ _program.Use() }, _fluidSphere.Mesh.Indices.size(), 0, _sim.m_particlePos.size());
        }

        // upload deformed positions
        int const nv = _softMesh.NumVertices();
        std::vector<glm::vec3> softPositions(nv);
        for (int i = 0; i < nv; ++i)
            softPositions[i] = glm::vec3(_softMesh.positions[i].x(), _softMesh.positions[i].y(), _softMesh.positions[i].z());
        auto spanVerts = Engine::make_span_bytes<glm::vec3>(softPositions);

        // compute per-vertex normals from surface faces
        std::vector<glm::vec3> normals(nv, glm::vec3(0.0f));
        for (auto const & f : _softMesh.surfaceFaces) {
            glm::vec3 const & p0 = softPositions[f[0]];
            glm::vec3 const & p1 = softPositions[f[1]];
            glm::vec3 const & p2 = softPositions[f[2]];
            glm::vec3 fn = glm::cross(p1 - p0, p2 - p0);
            normals[f[0]] += fn;
            normals[f[1]] += fn;
            normals[f[2]] += fn;
        }
        for (auto & n : normals) {
            float len = glm::length(n);
            if (len > 1e-10f) n /= len;
        }
        auto spanNormals = Engine::make_span_bytes<glm::vec3>(normals);

        // lit surface
        if (_showSurface) {
            _surfaceItem.UpdateVertexBuffer("position", spanVerts);
            if (_useLighting) {
                _litProgram.GetUniforms().SetByName("u_Projection", projMat);
                _litProgram.GetUniforms().SetByName("u_View", viewMat);
                _litProgram.GetUniforms().SetByName("u_ViewPosition", _sceneObject.Camera.Eye);
                _litProgram.GetUniforms().SetByName("u_Color", _surfaceColor);
                _litProgram.GetUniforms().SetByName("u_LightDir", glm::normalize(_lightDir));
                _litProgram.GetUniforms().SetByName("u_LightColor", _lightIntensity * glm::vec3(1.0f));
                _litProgram.GetUniforms().SetByName("u_AmbientColor", _ambientScale * _lightIntensity * glm::vec3(1.0f));
                _litProgram.GetUniforms().SetByName("u_Shininess", _shininess);
                _litProgram.GetUniforms().SetByName("u_FlatShading", int(_flatShading));
                _surfaceItem.UpdateVertexBuffer("normal", spanNormals);
                _surfaceItem.Draw({ _litProgram.Use() });
            } else {
                _lineProgram.GetUniforms().SetByName("u_Color", _surfaceColor);
                _lineProgram.GetUniforms().SetByName("u_Projection", projMat);
                _lineProgram.GetUniforms().SetByName("u_View", viewMat);
                _surfaceItem.Draw({ _lineProgram.Use() });
            }
        }

        // wireframe overlay
        if (_showWireframe) {
            glEnable(GL_LINE_SMOOTH);
            glLineWidth(0.5f);

            _lineProgram.GetUniforms().SetByName("u_Color", glm::vec3(0.85f, 0.35f, 0.2f));
            _lineProgram.GetUniforms().SetByName("u_Projection", projMat);
            _lineProgram.GetUniforms().SetByName("u_View", viewMat);
            _softEdgeItem.UpdateVertexBuffer("position", spanVerts);
            _softEdgeItem.Draw({ _lineProgram.Use() });

            glLineWidth(1.0f);
            glDisable(GL_LINE_SMOOTH);
        }

        // soft vertex spheres
        if (_showVertices) {
            _softColors.assign(nv, glm::vec3(0.9f, 0.6f, 0.35f));
            Rendering::ModelObject softParticles(_softSphere, softPositions, _softColors);
            softParticles.Mesh.Draw({ _program.Use() }, _softSphere.Mesh.Indices.size(), 0, nv);
        }

        DrawFluidSurface(projMat, viewMat);

        glEnable(GL_CULL_FACE);
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

} // namespace VCX::Labs::Coupling
