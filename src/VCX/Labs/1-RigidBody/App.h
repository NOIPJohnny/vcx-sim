#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/Common/UI.h"
#include "CaseRigidBody.h"

namespace VCX::Labs::RigidBody {
    class App : public VCX::Engine::IApp {
    private:
        Common::UI      _ui;
        CaseRigidBody   _caseRigidBody;
        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = {
            _caseRigidBody
        };

    public:
        App();

        void OnFrame() override;
    };
}