#pragma once
#include "beizier_planner/BeizierUtilities.hpp"


template<class T, unsigned int Dim>
BeizierCurves<T,Dim>::BeizierCurves(const std::array<T, Dim+1> &coeffs){
    coeffs_ = coeffs;
}

template<class T, unsigned int Dim>
void BeizierCurves<T,Dim>::DeCasteljauDecomposition(double t0,
                                    BeizierCurves<T,Dim>& b1,
                                    BeizierCurves<T,Dim>& b2){

    std::array<std::array<T, Dim+1>, Dim+1> ctmp;   

    for(int i(0); i<(Dim+1); ++i)
        ctmp[i][0] = coeffs_[i];

    for(int j(1); j<(Dim+1); ++j){
        for(int i(0); i< (Dim-j+1); ++j){
            ctmp[i][j] = ctmp[i][j-1]*(1-t0) + ctmp[i+1][j-1]*t0;
        }
    }
    
    for(int i(0); i<(Dim+1); ++i){
        b1.coeff_[i] = ctmp[0][i];
        b2.coeff_[i] = ctmp[i][Dim-i];
    }

}

wBeizier::wBeizier(){
    for(int i(0); i<10; ++i){
        by_.coeffs_[i] = Eigen::MatrixXd::Zero(6,3);
        bs_.coeffs_[i] = Eigen::VectorXd::Zero(6);
    }
}

wBeizier::wBeizier(const Eigen::Vector3d &com_init,
                   const Eigen::Vector3d &com_goal):wBeizier() {
                       

}

void wBeizier::decompose(const std::array<double, 4> &timeSequence, 
                        std::array<wBeizier, 3> &PwsSequence){

    double t1 = ( timeSequence[1]-timeSequence[0] )/(timeSequence[3]-timeSequence[0]);
    double t2 = ( timeSequence[2]-timeSequence[1] )/(timeSequence[3]-timeSequence[1]);

    wBeizier tmp;

    by_.DeCasteljauDecomposition(t1, PwsSequence[0].by_, tmp.by_);
    bs_.DeCasteljauDecomposition(t1, PwsSequence[0].bs_, tmp.bs_);

    tmp.by_.DeCasteljauDecomposition(t2, PwsSequence[1].by_, PwsSequence[2].by_);
    tmp.bs_.DeCasteljauDecomposition(t2, PwsSequence[1].bs_, PwsSequence[2].bs_);

}

// void wBeizier::decompose(const std::vector<double> &timeSequence, 
//                         std::vector<wBeizier> &PwsSequence){
    
// }