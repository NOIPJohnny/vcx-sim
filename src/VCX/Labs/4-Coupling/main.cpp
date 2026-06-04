#include "Assets/bundled.h"
#include "Labs/4-Coupling/App.h"

int main() {
    using namespace VCX;
    return Engine::RunApp<Labs::Coupling::App>(Engine::AppContextOptions {
        .Title      = "VCX-sim Labs 4: Coupling",
        .WindowSize = { 1024, 768 },
        .FontSize   = 16,

        .IconFileNames = Assets::DefaultIcons,
        .FontFileNames = Assets::DefaultFonts,
    });
}
