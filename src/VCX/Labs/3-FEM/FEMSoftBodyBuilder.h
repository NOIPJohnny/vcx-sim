#pragma once

namespace VCX::Labs::FEM {

    class FEMSystem;

    enum class SoftBodyType : int {
        GridBlock = 0,
    };

    int SoftBodyTypeCount();
    char const * SoftBodyTypeName(SoftBodyType type);
    void BuildSoftBodyStructure(FEMSystem & system, SoftBodyType type);

} // namespace VCX::Labs::FEM
