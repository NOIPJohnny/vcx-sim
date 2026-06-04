#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/Common/UI.h"
#include "CaseRBxFLIP.h"
#include "CaseFEMxFLIP.h"

namespace VCX::Labs::Coupling {
    class App : public VCX::Engine::IApp {
    private:
        Common::UI      _ui;
        CaseRBxFLIP   _caseRBxFLIP;
        CaseFEMxFLIP  _caseFEMxFLIP;
        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = {
            _caseRBxFLIP,
            _caseFEMxFLIP
        };

    public:
        App();

        void OnFrame() override;
    };
}
