#pragma once

#include <eigen3/Eigen/Dense>

template<unsigned Dim>
class BeizierCurves{
    public:
        BeizierCurves();
        ~BeizierCurves() {};

        void DeCasteljauDecomposition()


    public:
        std::array<Eigen::VecctorXd, Dim> coeffs_;
};

