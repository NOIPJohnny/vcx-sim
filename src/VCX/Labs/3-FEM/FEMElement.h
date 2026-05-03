#pragma once

#include <Eigen/Dense>
#include <array>

namespace VCX::Labs::FEM {

    struct TetElement {
        std::array<int, 4> indices; // vertex indices
        Eigen::Matrix3f    E_Inv;   // inverse of the rest shape matrix E
        float              volume;  // volume of the tetrahedron

        TetElement(int v0, int v1, int v2, int v3) : indices({v0, v1, v2, v3}), volume(0.0f) {
            E_Inv.setIdentity();
        }
    };

    struct TriElement {
        std::array<int, 3> indices;
        Eigen::Matrix2f E_Inv;      
        float area;                 
        
        TriElement(int v0, int v1, int v2) : indices({v0, v1, v2}), area(0.0f) {
            E_Inv.setIdentity();
        }
    };
} // namespace VCX::Labs::FEM