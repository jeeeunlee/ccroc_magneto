#include "beizier_planner/BeizierPlanner.hpp"
#include <iostream>
#include <stdio.h>

int main(int argc, char *argv[]) {

  std::string config_filename; 
  if(argc < 2) {
    config_filename = "config_magneto.yaml";        
  } else {
    config_filename = std::string(argv[1]);
  }
  config_filename.insert(0, THIS_COM);
  YAML::Node cfg = YAML::LoadFile(config_filename);


  std::cout<<"BeizierPlannerParameter - [[create]]"<<std::endl;
  BeizierPlannerParameter *param =
      new BeizierPlannerParameter(cfg);
  std::cout<<"BeizierPlanner - [[create]]"<<std::endl;
  BeizierPlanner *planner = new BeizierPlanner(param);

  // ===============================
  // Do planning
  // ===============================
  std::cout<<"DoPlan - [[start]]"<<std::endl;
  planner->DoPlan();

  std::cout << "[[Done]]" << std::endl;
  delete param;
  delete planner;
  
  return 0;
}
