#pragma once

#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Engine/Sphere.h"
#include "RigidBodySystem.h"
#include "RigidBody.h"
#include "Contact.h"
#include <vector>

namespace VCX::Labs::RigidBody {

    class CaseRigidBody : public Common::ICase {
        
    public:
        CaseRigidBody();

        virtual std::string_view const GetName() override { return "Rigid Body Simulation"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        RigidBodySystem _rigidBodySystem;
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-5, 5, 5) };
        VCX::Labs::Common::OrbitCameraManager _cameraManager;
        Engine::GL::UniqueIndexedRenderItem _boxItem;
        Engine::GL::UniqueIndexedRenderItem _sphereItem;
        Engine::GL::UniqueIndexedRenderItem _lineItem;

        bool _stopped { true };
        bool _gravityEnabled { false };
        int  _sceneId { 0 };
        int  _substeps { 10 };
        
        void OnProcessMouseControl(glm::vec3 mouseDelta);
        
        void ResetSystem();
        void SetupSceneSingle();
        void SetupSceneTwoBodies();
        void SetupSceneComplex();
        void SetupSceneNewtonPendulum();
        void SetupSceneSphere();
        void SetupSceneComplexSphere();

        // Interaction state
        int             _draggedBodyId { -1 };
        Eigen::Vector3f _dragTargetV { 0.f, 0.f, 0.f };
    };
} // namespace VCX::Labs::RigidBody