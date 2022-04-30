#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

template<class T, unsigned int Dim>
class BeizierCurves{
    public:
        BeizierCurves() {};
        BeizierCurves(const std::array<T, Dim+1> &coeffs);
        ~BeizierCurves() {};

        void DeCasteljauDecomposition(double t0,
                                    BeizierCurves<T, Dim>& b1,
                                    BeizierCurves<T, Dim>& b2);


    public:
        std::array<T, Dim+1> coeffs_;
};

class wBeizier{
    public:
    wBeizier();   
    wBeizier(const Eigen::Vector3d &com_init,
            const Eigen::Vector3d &com_goal);

    void decompose(const std::array<double, 4> &timeSequence, 
                    std::array<wBeizier, 3> &PwsSequence);

    // void decompose(const std::vector<double> &timeSequence, 
    //                 std::vector<wBeizier> &PwsSequence);

    ~wBeizier(){};

    public:
        BeizierCurves<Eigen::MatrixXd, 9> by_;
        BeizierCurves<Eigen::VectorXd, 9> bs_;
};

namespace beizier_utils{
    int factorial(int n) {
    if(n > 1)
        return n * factorial(n - 1);
    else
        return 1;
    }
}