#pragma once

#include "my_util/IOUtilities.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <ctime>

class cTimer{
  public:
  cTimer(){start = clock();end = start;}
  ~cTimer() {}
  void reset() {start = clock();end = start;}
  void print() {
      end = clock();
      cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;    
      std::cout<<" computation time =  "<< cpu_time_used<<std::endl; }
  void print_interval() {
      clock_t temp = clock();
      cpu_time_used = ((double) (temp - end)) / CLOCKS_PER_SEC;    
      std::cout<<" computation time =  "<< cpu_time_used<<std::endl; 
      end = temp;}


  protected:
  clock_t start;  
  clock_t end; 
  double cpu_time_used; 
};

template<class T, unsigned int Dim>
class BeizierCurves{
    public:
        BeizierCurves() {};
        BeizierCurves(const std::array<T, Dim+1> &coeffs);
        ~BeizierCurves() {};
        double getBeizierT(int i, double t);

        void DeCasteljauDecomposition(double t0,
                                    BeizierCurves<T, Dim>& b1,
                                    BeizierCurves<T, Dim>& b2);


    public:
        std::array<T, Dim+1> coeffs_;
};

class cBeizier{
public:
    cBeizier();   
    cBeizier(const Eigen::Vector3d &com_init,
            const Eigen::Vector3d &com_goal);

    Eigen::VectorXd getC(double t, const Eigen::Vector3d &y);

    // void decompose(const std::vector<double> &timeSequence, 
    //                 std::vector<wBeizier> &PwsSequence);

    ~cBeizier(){};

    public:
        BeizierCurves<Eigen::MatrixXd, 6> by_;
        BeizierCurves<Eigen::VectorXd, 6> bs_;
};

class wBeizier{
    public:
    wBeizier();   
    wBeizier(const Eigen::Vector3d &com_init,
            const Eigen::Vector3d &com_goal,
            const Eigen::Vector3d &gravity,
            double time_horizon);

    void decompose(const std::array<double, 4> &timeSequence, 
                    std::array<wBeizier, 3> &PwsSequence);

    Eigen::VectorXd getW(double t, const Eigen::Vector3d &y);

    // void decompose(const std::vector<double> &timeSequence, 
    //                 std::vector<wBeizier> &PwsSequence);

    ~wBeizier(){};

    public:
        BeizierCurves<Eigen::MatrixXd, 9> by_;
        BeizierCurves<Eigen::VectorXd, 9> bs_;
};

namespace beizier_utils{
    int factorial(int n);
    int combination(int n, int i);
    Eigen::MatrixXd friccone(double fcoeff);
    Eigen::MatrixXd skew(const Eigen::Vector3d& w);

    Eigen::MatrixXd hStack(const Eigen::MatrixXd& a_, const Eigen::MatrixXd& b_);
    Eigen::MatrixXd vStack(const Eigen::MatrixXd& a_, const Eigen::MatrixXd& b_);
    Eigen::MatrixXd hStack(const Eigen::VectorXd& a_, const Eigen::VectorXd& b_);
    Eigen::VectorXd vStack(const Eigen::VectorXd& a_, const Eigen::VectorXd& b_);
    Eigen::MatrixXd dStack(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b);
}