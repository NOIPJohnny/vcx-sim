#include "Labs/1-RigidBody/CaseRigidBody.h"
#include "Labs/Common/ImGuiHelper.h"
#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

namespace VCX::Labs::RigidBody {

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size())
        );
    }

    CaseRigidBody::CaseRigidBody() :
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) 
    {
        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 };
        _lineItem.UpdateElementBuffer(line_index);

        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        _boxItem.UpdateElementBuffer(tri_index);

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        ResetSystem();
    }

    void CaseRigidBody::ResetSystem() {
        _rigidBodySystem.Clear();
        _stopped = true;
        _gravityEnabled = false;
        _draggedBodyId = -1;
        _dragTargetV = Eigen::Vector3f::Zero();

        switch (_sceneId) {
            case 0: SetupSceneSingle(); break;
            case 1: SetupSceneTwoBodies(); break;
            case 2: SetupSceneComplex(); break;
        }
        
        if (_gravityEnabled) {
            _rigidBodySystem.EnableGravity();
        } else {
            _rigidBodySystem.DisableGravity();
        }
    }

    void CaseRigidBody::SetupSceneSingle() {
        RigidBodyItem body(Eigen::Vector3f(1.f, 2.f, 3.f), 10.0f, 
                           Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Quaternionf::Identity(),
                           Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 1.f, 0.f));
        _rigidBodySystem.AddBody(body);
        _gravityEnabled = false;
    }

    void CaseRigidBody::SetupSceneTwoBodies() {
        RigidBodyItem b1(Eigen::Vector3f(1.f, 1.f, 1.f), 1.0f, 
                         Eigen::Vector3f(-3.f, 0.f, 0.f), Eigen::Quaternionf::Identity(),
                         Eigen::Vector3f(2.f, 0.f, 0.f), Eigen::Vector3f(0.5f, 0.5f, 0.f));
                         
        RigidBodyItem b2(Eigen::Vector3f(1.f, 1.f, 1.f), 1.0f, 
                         Eigen::Vector3f(3.f, 0.f, 0.f), Eigen::Quaternionf::Identity(),
                         Eigen::Vector3f(-2.f, 0.f, 0.f), Eigen::Vector3f(0.f, -0.5f, 0.5f));
                         
        _rigidBodySystem.AddBody(b1);
        _rigidBodySystem.AddBody(b2);
        _gravityEnabled = false;
    }

    void CaseRigidBody::SetupSceneComplex() {
        RigidBodyItem floor(Eigen::Vector3f(10.f, 0.5f, 10.f), 0, 
                            Eigen::Vector3f(0.f, -2.f, 0.f));
        _rigidBodySystem.AddBody(floor);

        for (int i = 0; i < 4; ++i) {
            RigidBodyItem box(Eigen::Vector3f(1.f, 1.f, 1.f), 1.0f, 
                              Eigen::Vector3f(0.f, 2.f + i * 2.1f, 0.1f * i));
            _rigidBodySystem.AddBody(box);
        }
        _gravityEnabled = true;
    }

    void CaseRigidBody::OnSetupPropsUI() {
        if (ImGui::Combo("Scene", &_sceneId, "Single Body\0Two Bodies Collision\0Complex Gravity Scene\0"))
            ResetSystem();
        if (ImGui::Button("Reset System")) ResetSystem();
        ImGui::SameLine();
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
        if (ImGui::Checkbox("Enable Gravity", &_gravityEnabled)) {
            if (_gravityEnabled) _rigidBodySystem.EnableGravity();
            else _rigidBodySystem.DisableGravity();
        }
        ImGui::SliderInt("Substeps", &_substeps, 1, 20);
        float restitution = _rigidBodySystem.GetRestitution();
        float friction = _rigidBodySystem.GetFriction();
        ImGui::SliderFloat("Restitution", &restitution, 0.01f, 1.f);
        _rigidBodySystem.SetRestitution(restitution);
        ImGui::SliderFloat("Friction", &friction, 0.01f, 1.f);
        _rigidBodySystem.SetFriction(friction);
        ImGui::Spacing();
    }

    VCX::Labs::Common::CaseRenderResult CaseRigidBody::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        
        OnProcessMouseControl(_cameraManager.getMouseMove());

        if (!_stopped) {
            float dt = 1.0f / 60.0f; 
            float subdt = dt / static_cast<float>(_substeps);
            for(int i = 0; i < _substeps; ++i) {
                if (_sceneId == 2 && _draggedBodyId != -1 && _draggedBodyId < _rigidBodySystem.GetBodies().size()) {
                    auto& body = _rigidBodySystem.GetBodies()[_draggedBodyId];
                    Eigen::Vector3f override_force = (_dragTargetV - body.Getv()) * body.GetMass() / subdt;
                    if (_gravityEnabled && body.GetMass() > 0.0f) {
                        override_force -= Eigen::Vector3f(0, -9.8f, 0) * body.GetMass();
                    }
                    body.ApplyForce(override_force); 
                }

                _rigidBodySystem.Update(subdt);
            }
        }
        
        _frame.Resize(desiredSize);
        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);

        auto const& bodies = _rigidBodySystem.GetBodies();
        for (size_t i = 0; i < bodies.size(); ++i) {
            auto const& body = bodies[i];
            
            Eigen::Vector3f center = body.GetPosition();
            Eigen::Quaternionf q = body.GetOrient();
            Eigen::Vector3f dim = body.GetDim() / 2.0f; // Half lengths

            std::vector<glm::vec3> VertsPosition;
            VertsPosition.resize(8);

            Eigen::Vector3f hX(dim[0], 0, 0);
            Eigen::Vector3f hY(0, dim[1], 0);
            Eigen::Vector3f hZ(0, 0, dim[2]);

            Eigen::Vector3f corners[8] = {
                -hX + hY + hZ,  hX + hY + hZ,
                 hX + hY - hZ, -hX + hY - hZ,
                -hX - hY + hZ,  hX - hY + hZ,
                 hX - hY - hZ, -hX - hY - hZ
            };

            for (int k = 0; k < 8; ++k) {
                Eigen::Vector3f worldPos = center + q * corners[k];
                VertsPosition[k] = glm::vec3(worldPos.x(), worldPos.y(), worldPos.z());
            }

            auto span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);

            glm::vec3 Color = glm::vec3(121.0f, 207.0f, 171.0f) / 255.0f;
            if (i > 0) Color = glm::vec3(207.0f, 140.0f, 120.0f) / 255.0f;
            
            _program.GetUniforms().SetByName("u_Color", Color);
            _boxItem.UpdateVertexBuffer("position", span_bytes);
            _boxItem.Draw({ _program.Use() });

            _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
            _lineItem.UpdateVertexBuffer("position", span_bytes);
            _lineItem.Draw({ _program.Use() });
        }

        glDisable(GL_LINE_SMOOTH);

        return VCX::Labs::Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseRigidBody::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseRigidBody::OnProcessMouseControl(glm::vec3 mouseDelta) {
        if (_rigidBodySystem.GetBodies().empty()) return;
        bool isMouseDown = ImGui::IsMouseDown(ImGuiMouseButton_Left);

        if (_sceneId == 0) {
            float forceScale = 500.0f;
            auto& firstBody = _rigidBodySystem.GetBodies()[0];
            if (glm::length(mouseDelta) > 0.01f) {
                Eigen::Vector3f push_force(mouseDelta.x * forceScale, mouseDelta.y * forceScale, mouseDelta.z * forceScale);
                firstBody.ApplyForce(push_force); 
            }
        } 
        else if (_sceneId == 1)
            return;
        else if (_sceneId == 2) {
            if (!isMouseDown) {
                _draggedBodyId = -1;
                return;
            }

            auto& bodies = _rigidBodySystem.GetBodies();
            
            if (_draggedBodyId == -1) {
                float minDist = std::numeric_limits<float>::max();
                Eigen::Vector3f eye(_camera.Eye.x, _camera.Eye.y, _camera.Eye.z);
                for (size_t i = 0; i < bodies.size(); ++i) {
                    if (bodies[i].GetMass() <= 0) continue;
                    float dist = (bodies[i].GetPosition() - eye).norm();
                    if (dist < minDist) {
                        minDist = dist;
                        _draggedBodyId = i;
                    }
                }
            }
            if (_draggedBodyId != -1) {
                float speedScale = 20.0f; 
                _dragTargetV = Eigen::Vector3f(mouseDelta.x * speedScale, mouseDelta.y * speedScale, mouseDelta.z * speedScale);
            }
        }
    }
} // namespace VCX::Labs::RigidBody