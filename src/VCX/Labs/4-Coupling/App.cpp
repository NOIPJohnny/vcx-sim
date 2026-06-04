#include "Labs/4-Coupling/App.h"

namespace VCX::Labs::Coupling {
    App::App():
        _ui(
            Labs::Common::UIOptions { }) {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}