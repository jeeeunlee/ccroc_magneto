#pragma once

#include "my_util/IOUtilities.hpp"
#include <Configuration.h>

using namespace myUtils;


class BeizierPlannerParameter{
  public:
  BeizierPlannerParameter(YAML::Node planner_cfg);
  ~BeizierPlannerParameter() {};

  void UpdateContactPlan(YAML::Node cnt_cfg);
  void UpdateTrajectoryPlan(YAML::Node planner_cfg);


};

class BeizierPlanner{
  public:
  BeizierPlanner(BeizierPlannerParameter *_param);
  ~BeizierPlanner();
  void DoPlan();
  protected:
  BeizierPlannerParameter* mPlanParam;
};