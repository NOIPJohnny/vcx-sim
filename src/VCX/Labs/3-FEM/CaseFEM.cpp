#include "Labs/3-FEM/CaseFEM.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include <glm/gtc/matrix_transform.hpp>

#include <imgui.h>

#include "Engine/GL/VertexLayout.hpp"
#include "Engine/prelude.hpp"
#include "Labs/3-FEM/Integrator.h"

namespace VCX::Labs::FEM {

    CaseFEM::CaseFEM() :
        _program(Engine::GL::UniqueProgram({
            Engine::GL::SharedShader("assets/shaders/fluid.vert"),
            Engine::GL::SharedShader("assets/shaders/fluid.frag"),
        })),
        _lineProgram(Engine::GL::UniqueProgram({
            Engine::GL::SharedShader("assets/shaders/flat.vert"),
            Engine::GL::SharedShader("assets/shaders/flat.frag"),
        })),
        _particleItem(
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Static, 0)
                .Add<glm::vec3>("normal", Engine::GL::DrawFrequency::Static, 1)
                .Add<glm::vec3>("offset", Engine::GL::DrawFrequency::Stream, 2)
                .Add<glm::vec3>("color", Engine::GL::DrawFrequency::Stream, 3),
            Engine::GL::PrimitiveType::Triangles
        ),
        _edgeItem(
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
            Engine::GL::PrimitiveType::Lines
        ),
        _particleModel(Engine::Model { Engine::Sphere(6, 0.045f), 0 }),
        _sceneObject(1) {

        _particleItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_particleModel.Mesh.Positions));
        _particleItem.UpdateVertexBuffer("normal", Engine::make_span_bytes<glm::vec3>(_particleModel.Mesh.Normals));
        _particleItem.UpdateElementBuffer(_particleModel.Mesh.Indices);
        _particleItem.SetAttributeDivisor(2, 1);
        _particleItem.SetAttributeDivisor(3, 1);

        _program.BindUniformBlock("PassConstants", 1);
        _program.GetUniforms().SetByName("u_AmbientScale", 1.0f);
        _program.GetUniforms().SetByName("u_UseBlinn", 1);
        _program.GetUniforms().SetByName("u_Shininess", 32.0f);
        _program.GetUniforms().SetByName("u_UseGammaCorrection", 1);
        _program.GetUniforms().SetByName("u_AttenuationOrder", 2);
        _lineProgram.GetUniforms().SetByName("u_Color", glm::vec3(0.85f, 0.92f, 1.0f));

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        ResetSystem();
    }

    void CaseFEM::OnSetupPropsUI() {
        if (ImGui::Button("Reset System"))
            ResetSystem();
        ImGui::SameLine();
        if (ImGui::Button(_paused ? "Start" : "Pause"))
            _paused = ! _paused;
        ImGui::SameLine();
        if (ImGui::Button("Step Once"))
            _stepOnce = true;

        ImGui::Spacing();
        ImGui::SliderFloat("Fixed dt", &_fixedDt, 1e-4f, 1.0f / 60.0f, "%.5f");
        ImGui::SliderInt("Substeps", &_substeps, 1, 20);

        ImGui::Spacing();
        ImGui::SliderFloat("Young's Modulus", &_system.E, 100.0f, 50000.0f, "%.0f");
        ImGui::SliderFloat("Poisson Ratio", &_system.nu, 0.0f, 0.45f, "%.2f");
        ImGui::SliderFloat("Density", &_system.density, 50.0f, 2000.0f, "%.0f");
        ImGui::SliderFloat("Gravity", &_system.gravity.y, -10.0f, 1.0f, "%.3f");
        ImGui::SliderFloat("Damping", &_system.damping, 0.0f, 5.0f, "%.2f");

        ImGui::Spacing();
        int model = static_cast<int>(_elasticModel);
        if (ImGui::Combo("Elastic Model", &model, "Linear\0StVK\0Neo-Hookean\0Corotated\0")) {
            _elasticModel = static_cast<ElasticModel>(model);
            ApplyModel();
        }

        int integrator = static_cast<int>(_integratorMode);
        if (ImGui::Combo("Integrator", &integrator, "Explicit\0Implicit\0")) {
            _integratorMode = static_cast<IntegratorMode>(integrator);
            ApplyIntegrator();
        }

        if (_integratorMode == IntegratorMode::Implicit) {
            ImGui::SliderInt("Newton Iters", &_newtonIters, 1, 30);
            ImGui::InputFloat("Newton Tolerance", &_newtonTolerance, 0.0f, 0.0f, "%.1e");
            if (auto * implicit = dynamic_cast<ImplicitIntegrator *>(_system.integrator.get())) {
                implicit->maxIters = _newtonIters;
                implicit->tol = std::max(_newtonTolerance, 1e-8f);
            }
        }

        ImGui::Spacing();
        ImGui::Text("Particles: %d", static_cast<int>(_system.positions.size()));
        ImGui::Text("Tets: %d", static_cast<int>(_system.tets.size()));
        ImGui::Text("Current dt: %.5f", _lastStepDt);
    }

    Common::CaseRenderResult CaseFEM::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        _viewportSize = desiredSize;
        _frame.Resize(desiredSize);
        _cameraManager.Update(_camera);

        bool const shouldStep = (! _paused) || _stepOnce;
        _lastStepDt = _fixedDt;

        if (shouldStep) {
            ApplyMouseForce(_lastMousePos);
            StepSimulation(_fixedDt);
            std::fill(_system.externalForces.begin(), _system.externalForces.end(), glm::vec3(0.0f));
            _stepOnce = false;
        }

        UpdateRenderData();

        glm::mat4 const projection = _camera.GetProjectionMatrix(float(desiredSize.first) / float(desiredSize.second));
        glm::mat4 const view = _camera.GetViewMatrix();

        Rendering::SceneObject::PassConstants pass {};
        pass.Projection = projection;
        pass.View = view;
        pass.ViewPosition = _camera.Eye;
        pass.AmbientIntensity = glm::vec3(0.65f);
        pass.CntPointLights = 0;
        pass.CntSpotLights = 0;
        pass.CntDirectionalLights = 0;
        _sceneObject.PassConstantsBlock.Update(pass);

        _lineProgram.GetUniforms().SetByName("u_Projection", projection);
        _lineProgram.GetUniforms().SetByName("u_View", view);

        _particleOffsets = _system.positions;
        _particleColors.assign(_system.positions.size(), glm::vec3(0.9f, 0.3f, 0.3f));
        if (_controlledVertex >= 0 && _controlledVertex < static_cast<int>(_particleColors.size()))
            _particleColors[_controlledVertex] = glm::vec3(1.0f, 0.9f, 0.15f);
        _particleItem.UpdateVertexBuffer("offset", Engine::make_span_bytes<glm::vec3>(_particleOffsets));
        _particleItem.UpdateVertexBuffer("color", Engine::make_span_bytes<glm::vec3>(_particleColors));

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.06f, 0.07f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glLineWidth(1.5f);
        _edgeItem.Draw({ _lineProgram.Use() });
        glLineWidth(1.0f);

        _particleItem.Draw(
            { _program.Use() },
            _particleModel.Mesh.Indices.size(),
            0,
            _system.positions.size()
        );

        glDisable(GL_DEPTH_TEST);

        return Common::CaseRenderResult {
            .Fixed = false,
            .Flipped = true,
            .Image = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseFEM::OnProcessInput(ImVec2 const & pos) {
        _lastMousePos = pos;
        _hasLastMousePos = true;
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseFEM::ResetSystem() {
        ApplyModel();
        ApplyIntegrator();
        _system.ResetSystem();
        _hasLastMousePos = false;
        UpdateTetEdgeIndices();
        UpdateRenderData();
    }

    void CaseFEM::ApplyModel() {
        switch (_elasticModel) {
        case ElasticModel::Linear:
            _system.model = std::make_unique<LinearModel>();
            break;
        case ElasticModel::StVK:
            _system.model = std::make_unique<StVKModel>();
            break;
        case ElasticModel::NeoHookean:
            _system.model = std::make_unique<NeoHookeanModel>();
            break;
        case ElasticModel::Corotated:
            _system.model = std::make_unique<CorotatedModel>();
            break;
        }
    }

    void CaseFEM::ApplyIntegrator() {
        if (_integratorMode == IntegratorMode::Implicit) {
            auto implicit = std::make_unique<ImplicitIntegrator>();
            implicit->maxIters = _newtonIters;
            implicit->tol = std::max(_newtonTolerance, 1e-8f);
            _system.integrator = std::move(implicit);
        } else
            _system.integrator = std::make_unique<ExplicitIntegrator>();
    }

    void CaseFEM::StepSimulation(float dt) {
        int const substeps = std::max(_substeps, 1);
        float const h = dt / static_cast<float>(substeps);
        for (int i = 0; i < substeps; ++i) {
            _system.Update(h);
        }
    }

    void CaseFEM::ApplyMouseForce(ImVec2 const &) {
        std::fill(_system.externalForces.begin(), _system.externalForces.end(), glm::vec3(0.0f));
        _controlledVertex = -1;

        auto const & io = ImGui::GetIO();
        if (!_hasLastMousePos || !io.KeyAlt || !ImGui::IsMouseDown(ImGuiMouseButton_Left))
            return;

        ImVec2 const delta = io.MouseDelta;
        float const speedPx = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        if (speedPx < 0.01f)
            return;

        std::uint32_t const width = std::max<std::uint32_t>(_viewportSize.first, 1);
        std::uint32_t const height = std::max<std::uint32_t>(_viewportSize.second, 1);
        glm::mat4 const view = _camera.GetViewMatrix();
        glm::mat4 const projection = _camera.GetProjectionMatrix(float(width) / float(height));
        glm::mat4 const viewProjection = projection * view;

        glm::vec3 const forward = glm::normalize(_camera.Target - _camera.Eye);
        glm::vec3 const right = glm::normalize(glm::cross(forward, _camera.Up));
        glm::vec3 const up = glm::normalize(glm::cross(right, forward));
        glm::vec3 const forceDir = right * delta.x + up * -delta.y;
        float const forceDirLen = glm::length(forceDir);
        if (forceDirLen < 1e-6f)
            return;

        glm::vec3 const force = glm::normalize(forceDir) * std::min(speedPx * _mouseForceScale, _mouseMaxForce);
        glm::vec2 const mouse(_lastMousePos.x, _lastMousePos.y);
        int nearestVertex = -1;
        float nearestDist2 = std::numeric_limits<float>::max();

        for (std::size_t i = 0; i < _system.positions.size(); ++i) {
            if (_system.fixed[i])
                continue;

            glm::vec4 const clip = viewProjection * glm::vec4(_system.positions[i], 1.0f);
            if (clip.w <= 1e-6f)
                continue;

            glm::vec3 const ndc = glm::vec3(clip) / clip.w;
            if (ndc.z < -1.0f || ndc.z > 1.0f)
                continue;

            glm::vec2 const screen(
                (ndc.x * 0.5f + 0.5f) * float(width),
                (0.5f - ndc.y * 0.5f) * float(height)
            );
            glm::vec2 const d = screen - mouse;
            float const dist2 = glm::dot(d, d);
            if (dist2 < nearestDist2) {
                nearestDist2 = dist2;
                nearestVertex = static_cast<int>(i);
            }
        }

        if (nearestVertex >= 0) {
            _controlledVertex = nearestVertex;
            _system.externalForces[nearestVertex] += force;
        }
    }

    void CaseFEM::UpdateRenderData() {
        _edgeVertices.clear();
        _edgeVertices.reserve(_edgeIndices.size());

        for (auto const & tet : _system.tets) {
            int const ids[4] = { tet.indices[0], tet.indices[1], tet.indices[2], tet.indices[3] };
            int const edges[6][2] = {
                {0, 1}, {0, 2}, {0, 3},
                {1, 2}, {1, 3}, {2, 3},
            };
            for (auto const & edge : edges) {
                _edgeVertices.push_back(_system.positions[ids[edge[0]]]);
                _edgeVertices.push_back(_system.positions[ids[edge[1]]]);
            }
        }

        _edgeItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_edgeVertices));
    }

    void CaseFEM::UpdateTetEdgeIndices() {
        _edgeIndices.resize(_system.tets.size() * 12);
        for (std::size_t i = 0; i < _edgeIndices.size(); ++i)
            _edgeIndices[i] = static_cast<std::uint32_t>(i);
        _edgeItem.UpdateElementBuffer(_edgeIndices);
    }

} // namespace VCX::Labs::FEM
