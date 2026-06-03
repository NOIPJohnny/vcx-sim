#include "Assets/bundled.h"
#include "Labs/4-RBxFLIP/App.h"

int main() {
    using namespace VCX;
    return Engine::RunApp<Labs::RBxFLIP::App>(Engine::AppContextOptions {
        .Title      = "VCX-sim Labs 4: RBxFLIP",
        .WindowSize = { 1024, 768 },
        .FontSize   = 16,

        .IconFileNames = Assets::DefaultIcons,
        .FontFileNames = Assets::DefaultFonts,
    });
}
