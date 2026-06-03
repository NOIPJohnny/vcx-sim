#include "Labs/4-RBxFLIP/App.h"

namespace VCX::Labs::RBxFLIP {
    App::App():
        _ui(
            Labs::Common::UIOptions { }) {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}