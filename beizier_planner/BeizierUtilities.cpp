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
        for(int i(0); i< (Dim-j+1); ++i){
            ctmp[i][j] = ctmp[i][j-1]*(1-t0) + ctmp[i+1][j-1]*t0;
        }
    }
    
    for(int i(0); i<(Dim+1); ++i){
        b1.coeffs_[i] = ctmp[0][i];
        b2.coeffs_[i] = ctmp[i][Dim-i];
    }

}

template<class T, unsigned int Dim>
double BeizierCurves<T,Dim>::getBeizierT(int i, double t){

    return (double)(beizier_utils::combination(Dim,i))*
                std::pow(t, i)*std::pow(t,Dim-i);    
}

wBeizier::wBeizier(){
    for(int i(0); i<10; ++i){
        by_.coeffs_[i] = Eigen::MatrixXd::Zero(6,3);
        bs_.coeffs_[i] = Eigen::VectorXd::Zero(6);
    }
}

wBeizier::wBeizier(const Eigen::Vector3d &com_init,
                   const Eigen::Vector3d &com_goal,
                   const Eigen::Vector3d &gravity,
                   double time_horizon):wBeizier() {

    // gravity = [0,0,-9.81]

    Eigen::Vector3d gravityT2 = gravity *time_horizon*time_horizon;

    Eigen::MatrixXd cs = Eigen::MatrixXd::Zero(6,3);


    cs.block(0,0,3,3) = Eigen::MatrixXd::Identity(3,3);
    cs.block(3,0,3,3) = beizier_utils::skew(com_init);

    by_.coeffs_[1] = 4./9.*cs;
    by_.coeffs_[2] = 2./9.*cs;
    by_.coeffs_[3] = -4./21.*cs;

    cs.block(3,0,3,3) = beizier_utils::skew(com_init-gravityT2);
    by_.coeffs_[4] = -10./21*cs;
    cs.block(3,0,3,3) = beizier_utils::skew(com_goal-gravityT2);
    by_.coeffs_[5] = -10./21*cs;
    cs.block(3,0,3,3) = beizier_utils::skew(com_goal);
    by_.coeffs_[6] = -4./21.*cs;
    by_.coeffs_[7] = 2./9.*cs;
    by_.coeffs_[8] = 4./9.*cs;

    bs_.coeffs_[0]<< gravityT2[0], gravityT2[1], -gravityT2[2], 
                -com_init[1]*gravityT2[2], com_init[0]*gravityT2[2], 0.;
    bs_.coeffs_[1]<< -4./9.*com_init[0], -4./9.*com_init[1], -4./9.*com_init[2]-gravityT2[2], 
                -com_init[1]*gravityT2[2], com_init[0]*gravityT2[2], 4./9.*com_init[0];

    bs_.coeffs_[2].segment(0,3) = 1./6.*com_goal -7./18.*com_init - gravityT2;
    bs_.coeffs_[2].segment(3,3) = 1./6.*beizier_utils::skew(com_init)*com_goal
                                    - beizier_utils::skew(com_init)*gravityT2;

    bs_.coeffs_[3].segment(0,3) = 13./42.*com_goal -5./42.*com_init - gravityT2;
    bs_.coeffs_[3].segment(3,3) = 13./42.*beizier_utils::skew(com_init)*com_goal
                                    - 16./21.*beizier_utils::skew(com_init)*gravityT2;

    bs_.coeffs_[4].segment(0,3) = 20./63.*com_goal +10./63.*com_init - gravityT2;
    bs_.coeffs_[4].segment(3,3) = 20./63.*beizier_utils::skew(com_init)*com_goal
                                    - 17./42.*beizier_utils::skew(com_init)*gravityT2
                                    - 5./42.*beizier_utils::skew(com_goal)*gravityT2;

    bs_.coeffs_[5].segment(0,3) = 10./63.*com_goal +20./63.*com_init - gravityT2;
    bs_.coeffs_[5].segment(3,3) = -20./63.*beizier_utils::skew(com_init)*com_goal
                                    - 5./42.*beizier_utils::skew(com_init)*gravityT2
                                    - 17./42.*beizier_utils::skew(com_goal)*gravityT2;

    bs_.coeffs_[6].segment(0,3) = 13./42.*com_init -5./42.*com_goal - gravityT2;
    bs_.coeffs_[6].segment(3,3) = 13./42.*beizier_utils::skew(com_goal)*com_init
                                    - 16./21.*beizier_utils::skew(com_goal)*gravityT2;

    bs_.coeffs_[7].segment(0,3) = 1./6.*com_init -7./18.*com_goal - gravityT2;
    bs_.coeffs_[7].segment(3,3) = 1./6.*beizier_utils::skew(com_goal)*com_init
                                    - beizier_utils::skew(com_goal)*gravityT2;
    
    bs_.coeffs_[8]<< -4./9.*com_goal[0], -4./9.*com_goal[1], -4./9.*com_goal[2]-gravityT2[2], 
                    -com_goal[1]*gravityT2[2], com_goal[0]*gravityT2[2], 4./9.*com_goal[0];

    bs_.coeffs_[9]<< gravityT2[0], gravityT2[1], -gravityT2[2], 
                    -com_goal[1]*gravityT2[2], com_goal[0]*gravityT2[2], 0.;

}

Eigen::VectorXd wBeizier::getW(double t, const Eigen::Vector3d &y){

    // w(t,y) = sum ( bi(t)*(biy*y+bis) )
    Eigen::VectorXd w = Eigen::VectorXd::Zero(6);
    for(int i(0); i<10; ++i){
        w = w+ by_.getBeizierT(i,t)*(by_.coeffs_[i]*y + bs_.coeffs_[i]);
    }
    return w;
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

namespace beizier_utils{
int factorial(int n) {
    if(n > 1)
        return n * factorial(n - 1);
    else
        return 1;
}

int combination(int n, int i){
    return factorial(n)/factorial(i)/factorial(n-i);
}

Eigen::MatrixXd friccone(double fcoeff){
    Eigen::MatrixXd cone_mat = Eigen::MatrixXd::Zero(5,3);
    cone_mat.row(0) << 1.0, 0.0, -fcoeff/sqrt(2.0);
    cone_mat.row(1) << -1.0, 0.0, -fcoeff/sqrt(2.0);
    cone_mat.row(2) << 0.0, -1.0, -fcoeff/sqrt(2.0);
    cone_mat.row(3) << 0.0, +1.0, -fcoeff/sqrt(2.0);
    cone_mat.row(4) << 0.0, +0.0, -1.0;
    return cone_mat;
}


Eigen::MatrixXd skew(const Eigen::Vector3d& w){
    // [[ 0, -3,  2],
    // [ 3,  0, -1],
    // [-2,  1,  0]]
    Eigen::MatrixXd Wx = Eigen::MatrixXd::Zero(3,3);
    Wx <<   0.0,   -w(2),  w(1),
            w(2),     0.0, -w(0),
            -w(1),    w(0),  0.0;
    return Wx;
}


Eigen::MatrixXd hStack(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
    if (a.rows()==0 || a.cols()==0)
        return b;
    if (b.rows()==0 || b.cols()==0)
        return a;
    if (a.rows() != b.rows()) {
        std::cout << "[hStack] Matrix Size is Wrong" << std::endl;
        exit(0);
    }

    Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows(), a.cols() + b.cols());
    ab << a, b;
    return ab;
}

Eigen::MatrixXd hStack(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    if (a.size()==0)
        return b;
    if (b.size()==0)
        return a;
    if (a.size() != b.size()) {
        std::cout << "[hStack] Vector Size is Wrong" << std::endl;
        exit(0);
    }
    Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.size(), 2);
    ab << a, b;
    return ab;
}

Eigen::MatrixXd vStack(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
    if (a.rows()==0 || a.cols()==0)
        return b;
    if (b.rows()==0 || b.cols()==0)
        return a;
    if (a.cols() != b.cols()) {
        std::cout << "[vStack] Matrix Size is Wrong" << std::endl;
        exit(0);
    }
    Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows() + b.rows(), a.cols());
    ab << a, b;
    return ab;
}

Eigen::VectorXd vStack(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    if(a.size()==0) return b;
    if(b.size()==0) return a;
    Eigen::VectorXd ab = Eigen::VectorXd::Zero(a.size()+b.size());    
    // ab.head( a.size() ) = a; 
    // ab.tail( b.size() ) = b;
    ab << a, b;
    return ab;
}

Eigen::MatrixXd dStack(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
    // diagonally stack a,b -> [a 0; 0 b]
    if (a.rows()==0 || a.cols()==0)
        return b;
    if (b.rows()==0 || b.cols()==0)
        return a;
    Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows() + b.rows(), a.cols() + b.cols());
    ab << a, Eigen::MatrixXd::Zero(a.rows(), b.cols()), 
        Eigen::MatrixXd::Zero(b.rows(), a.cols()), b;
    return ab;
}

}