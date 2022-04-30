#include "beizier_planner/BeizierPlanner.hpp"
#include "beizier_planner/RobotModel.hpp"
#include <ctime>

using namespace myUtils;


BeizierPlannerParameter::BeizierPlannerParameter(YAML::Node planner_cfg) {
  try {
    // =====================================================================
    // initial_robot_configuration Setting
    // =====================================================================
    std::cout<<"Initial_robot_configuration Setting"<<std::endl;
    UpdateTrajectoryPlan(planner_cfg);

    // =====================================================================
    // Contact Sequence Setting
    // =====================================================================
    std::cout<<"Contact Sequence Setting"<<std::endl;
    YAML::Node contact_plan = planner_cfg["contact_plan"];
    UpdateContactPlan(contact_plan);



  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }
}

void BeizierPlannerParameter::UpdateContactPlan(YAML::Node cnt_cfg){
  try { 
    Eigen::VectorXd _num_contacts(4);
    Eigen::VectorXd _cs_tmp(14); // contact sequence parameter
    readParameter(cnt_cfg, "num_contacts", _num_contacts);
    for (int eef_id=0; eef_id<RobotModel::numEEf; eef_id++) {
      contactPlanInterface.contactsPerEndeff[eef_id] = _num_contacts[eef_id];
      contactPlanInterface.contactSequence.eEfContacts[eef_id].clear();
      YAML::Node eff_params = cnt_cfg["eefcnt_" + RobotModel::eEfIdToString(eef_id)];
      for (int c_id = 0; c_id < _num_contacts[eef_id]; ++c_id) {
        readParameter(eff_params, "cnt"+std::to_string(c_id), _cs_tmp);
        contactPlanInterface.contactSequence.eEfContacts[eef_id].push_back(
          ContactState(_cs_tmp));

        std::cout << "c_id : " << c_id << std::endl;
        std::cout << contactPlanInterface.contactSequence.eEfContacts[eef_id][c_id]
                  << std::endl;
      }
    }
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
  }
}

void BeizierPlannerParameter::UpdateTrajectoryPlan(YAML::Node planner_cfg){
  YAML::Node inirobot_cfg = planner_cfg["initial_robot_configuration"];
  YAML::Node optim_cfg = planner_cfg["optimization"];
}


BeizierPlanner::BeizierPlanner(BeizierPlannerParameter *_param) {
  mPlanParam = _param;
}

BeizierPlanner::~BeizierPlanner() {}

void BeizierPlanner::DoPlan(){

}