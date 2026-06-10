#pragma once

namespace VCX::Labs::FEM {

    class FEMSystem;

    enum class SoftBodyType : int {
        GridBlock = 0,
        Sphere = 1,
        TeddyBear = 2,
    };

    int SoftBodyTypeCount();
    char const * SoftBodyTypeName(SoftBodyType type);
    void BuildSoftBodyStructure(FEMSystem & system, SoftBodyType type);

} // namespace VCX::Labs::FEM
