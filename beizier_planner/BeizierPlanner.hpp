#pragma once

#include "my_util/IOUtilities.hpp"
#include "beizier_planner/RobotModel.hpp"
#include "beizier_planner/ContactState.hpp"
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

  Eigen::Vector3d comStart_, comGoal_;

};

class BeizierPlanner{
  public:
  BeizierPlanner(BeizierPlannerParameter *_param);
  ~BeizierPlanner();
  void DoPlan();
  protected:
  BeizierPlannerParameter* mPlanParam;
};