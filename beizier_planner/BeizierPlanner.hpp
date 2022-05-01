#pragma once
#pragma GCC diagnostic ignored "-Wunused-variable"

#include "my_util/IOUtilities.hpp"
#include "my_qpsolver/QuadProgSolver.hpp"
#include "beizier_planner/RobotModel.hpp"
#include "beizier_planner/ContactState.hpp"
#include "beizier_planner/BeizierUtilities.hpp"
#include <Configuration.h>

using namespace myUtils;


class BeizierPlannerParameter{
  public:
  BeizierPlannerParameter(YAML::Node planner_cfg);
  ~BeizierPlannerParameter() {};

  void UpdateContactPlan(YAML::Node cnt_cfg);
  void UpdateTrajectoryPlan(YAML::Node planner_cfg);

  public:
  ContactStateSequence contactSequence_;
  Eigen::Matrix<int, RobotModel::numEEf, 1> contactsPerEndeff_;

  double gravity_, robotMass_, timeHorizon_;
  int numActEEfs_;

  Eigen::Vector3d comStart_, comGoal_, gravVec_;

};

class BeizierPlanner{
  public:
    BeizierPlanner(BeizierPlannerParameter *_param);
    ~BeizierPlanner();
    
    void DoPlan();    


  protected:
    void initialize();

    void setBeizierCurve();
    void buildAMatrices();
    void buildFrictionCones();
    void solveDD();
    void buildInequalities();
    void solveQP();
    void test();
    

  protected:
    cTimer timer_;
    QuadProgSolver* qp_solver_;
    

    BeizierPlannerParameter* mPlanParam;
    
    std::array<Eigen::MatrixXd, 3> ASequence_;
    std::array<Eigen::MatrixXd, 3> FricConeSequence_;
    std::array<Eigen::VectorXd, 3> FmSequence_;
    std::array<Eigen::MatrixXd, 3> UavSequence_;

    int moving_eef_id_;

    wBeizier Pws_;
    std::array<wBeizier, 3> PwsSequence_;
    std::array<double, 4> timeSequence_;

    Eigen::MatrixXd H_;
    Eigen::VectorXd h_;

};