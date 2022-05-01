#include "beizier_planner/BeizierPlanner.hpp"
#include <my_geometry/include/my_geometry/Polytope/Polytope.h>



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
      contactsPerEndeff_[eef_id] = _num_contacts[eef_id];
      contactSequence_.eEfContacts[eef_id].clear();
      YAML::Node eff_params = cnt_cfg["eefcnt_" + RobotModel::eEfIdToString(eef_id)];
      for (int c_id = 0; c_id < _num_contacts[eef_id]; ++c_id) {
        readParameter(eff_params, "cnt"+std::to_string(c_id), _cs_tmp);
        contactSequence_.eEfContacts[eef_id].push_back(
          ContactState(_cs_tmp));

        // std::cout << "c_id : " << c_id << std::endl;
        // std::cout << contactSequence_.eEfContacts[eef_id][c_id]
        //           << std::endl;
      }
    }
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
  }
}

void BeizierPlannerParameter::UpdateTrajectoryPlan(YAML::Node planner_cfg){
  YAML::Node inirobot_cfg = planner_cfg["initial_robot_configuration"];
  YAML::Node optim_cfg = planner_cfg["optimization"];

  try { 
    readParameter(inirobot_cfg, "com", comStart_);
    readParameter(optim_cfg, "com_goal", comGoal_);    

    readParameter(optim_cfg, "time_horizon", timeHorizon_);
    readParameter(optim_cfg, "n_act_eefs", numActEEfs_);

    readParameter(optim_cfg, "robot_mass", robotMass_);
    readParameter(optim_cfg, "gravity", gravity_); 

    gravVec_ << 0., 0., -gravity_;  
      
    
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
  }


}


BeizierPlanner::BeizierPlanner(BeizierPlannerParameter *_param) {
  mPlanParam = _param;
  qp_solver_ = new QuadProgSolver();
  initialize();
}

BeizierPlanner::~BeizierPlanner() {}


void BeizierPlanner::initialize(){

  // find swing foot
  moving_eef_id_ = 0;
  for (int eef_id=0; eef_id<RobotModel::numEEf; eef_id++){
    if( mPlanParam->contactsPerEndeff_[eef_id] == 2 ) moving_eef_id_ = eef_id;
  }
  std::cout<<"moving_eef_id_ = "<< moving_eef_id_ <<std::endl;

  // set timeset
  double t0 = mPlanParam->contactSequence_.eEfContacts[moving_eef_id_][0].timeIni;
  double t1 = mPlanParam->contactSequence_.eEfContacts[moving_eef_id_][0].timeEnd;
  double t2 = mPlanParam->contactSequence_.eEfContacts[moving_eef_id_][1].timeIni;
  double t3 = mPlanParam->timeHorizon_;
  // double t3 = mPlanParam->contactSequence_.eEfContacts[moving_eef_id_][1].timeEnd;
  timeSequence_ = {t0,t1,t2,t3};
  std::cout<<"timeSequence_ = "<< t0 <<","<<t1<<","<<t2<<","<<t3<<std::endl;
}

void BeizierPlanner::DoPlan(){
  
  //  

  timer_.reset();
  setBeizierCurve();
  timer_.print_interval();

  // update ASequence_
  buildAMatrices();
  timer_.print_interval();

  //update FricConeSequence_, FmSequence_
  buildFrictionCones();
  timer_.print_interval();

  // solve DD
  solveDD();
  timer_.print_interval();

  // buildInequalities
  buildInequalities();
  timer_.print_interval();

  solveQP();
  timer_.print_interval();
  timer_.print();


}

void BeizierPlanner::setBeizierCurve(){
  Pws_ = wBeizier(mPlanParam->comStart_, mPlanParam->comGoal_, mPlanParam->gravVec_, mPlanParam->timeHorizon_);
  Pws_.decompose(timeSequence_, PwsSequence_); 
}

void BeizierPlanner::buildAMatrices(){
  
  // initialize
  ASequence_[0] = Eigen::MatrixXd::Zero(6,3*RobotModel::numEEf); //full
  ASequence_[1] = Eigen::MatrixXd::Zero(6,3*(RobotModel::numEEf-1)); //swing
  ASequence_[2] = Eigen::MatrixXd::Zero(6,3*RobotModel::numEEf); //full

  Eigen::Vector3d pi;
  Eigen::Matrix3d Ri;
  // just assume 3 contact sequence (full->swing->full)
  int tmp_id=0;
  for (int eef_id=0; eef_id<RobotModel::numEEf; eef_id++){
    
    pi = mPlanParam->contactSequence_.eEfContacts[eef_id][0].position;
    Ri = mPlanParam->contactSequence_.eEfContacts[eef_id][0].orientation.toRotationMatrix();

    // seq 1 (full)
    ASequence_[0].block(0,3*eef_id,3,3 ) = Ri;
    ASequence_[0].block(3,3*eef_id,3,3 ) = beizier_utils::skew(pi)*Ri;
    // seq 2 (swing)
    if(moving_eef_id_ != eef_id){
      ASequence_[1].block(0,3*tmp_id,3,3 ) = Ri;
      ASequence_[1].block(3,3*tmp_id,3,3 ) = beizier_utils::skew(pi)*Ri;
      tmp_id++;
    }
    // seq 3 (full)
    if(moving_eef_id_ == eef_id){
      pi = mPlanParam->contactSequence_.eEfContacts[eef_id][1].position;
      Ri = mPlanParam->contactSequence_.eEfContacts[eef_id][1].orientation.toRotationMatrix();
    }
    ASequence_[2].block(0,3*eef_id,3,3 ) = Ri;
    ASequence_[2].block(3,3*eef_id,3,3 ) = beizier_utils::skew(pi)*Ri;
  }

  // std::cout<<"ASequence_[0]="<<ASequence_[0]<<std::endl;
  // std::cout<<"ASequence_[1]="<<ASequence_[1]<<std::endl;
  // std::cout<<"ASequence_[2]="<<ASequence_[2]<<std::endl;

}

void BeizierPlanner::buildFrictionCones(){
  int fricConeDim = 5;
    // initialize
  FricConeSequence_[0] = Eigen::MatrixXd::Zero(fricConeDim*RobotModel::numEEf,3*RobotModel::numEEf); //full
  FricConeSequence_[1] = Eigen::MatrixXd::Zero(fricConeDim*(RobotModel::numEEf-1),3*(RobotModel::numEEf-1)); //swing
  FricConeSequence_[2] = Eigen::MatrixXd::Zero(fricConeDim*RobotModel::numEEf,3*RobotModel::numEEf); //full

  FmSequence_[0] = Eigen::VectorXd::Zero(3*RobotModel::numEEf);
  FmSequence_[1] = Eigen::VectorXd::Zero(3*(RobotModel::numEEf-1));
  FmSequence_[2] = Eigen::VectorXd::Zero(3*RobotModel::numEEf);

  double mu;
  Eigen::Vector3d fm;
  // just assume 3 contact sequence (full->swing->full)
  int tmp_id=0;
  for (int eef_id=0; eef_id<RobotModel::numEEf; eef_id++){
    
    mu = mPlanParam->contactSequence_.eEfContacts[eef_id][0].fricCoeff;
    fm = mPlanParam->contactSequence_.eEfContacts[eef_id][0].frcMag;


    // seq 1 (full)
    FricConeSequence_[0].block(fricConeDim*eef_id,3*eef_id,fricConeDim,3 ) = beizier_utils::friccone(mu);
    FmSequence_[0].segment(3*eef_id, 3) = -fm;

    // seq 2 (swing)
    if(moving_eef_id_ != eef_id){
      FricConeSequence_[1].block(fricConeDim*tmp_id,3*tmp_id,fricConeDim,3 ) = beizier_utils::friccone(mu);
      FmSequence_[1].segment(3*tmp_id, 3) = -fm;
      tmp_id++;
    }
    // seq 3 (full)
    if(moving_eef_id_ == eef_id){
      mu = mPlanParam->contactSequence_.eEfContacts[eef_id][1].fricCoeff;
      fm = mPlanParam->contactSequence_.eEfContacts[eef_id][1].frcMag;
    }
    FricConeSequence_[2].block(fricConeDim*eef_id,3*eef_id,fricConeDim,3 ) = beizier_utils::friccone(mu);
    FmSequence_[2].segment(3*eef_id, 3) = -fm;
  }

  // std::cout<<"FricConeSequence_[0]="<<FricConeSequence_[0]<<std::endl;
  // std::cout<<"FricConeSequence_[1]="<<FricConeSequence_[1]<<std::endl;
  // std::cout<<"FricConeSequence_[2]="<<FricConeSequence_[2]<<std::endl;

  // std::cout<<"FmSequence_[0]="<<FmSequence_[0]<<std::endl;
  // std::cout<<"FmSequence_[1]="<<FmSequence_[1]<<std::endl;
  // std::cout<<"FmSequence_[2]="<<FmSequence_[2]<<std::endl;
}

void BeizierPlanner::solveDD(){
  
  Polyhedron poly_1, poly_2;
  for(int cid(0); cid<3; cid++){
    // U -> V
    int n_fc = FricConeSequence_[cid].rows();
    bool b_poly_Uf = poly_1.setHrep(FricConeSequence_[cid], Eigen::VectorXd::Zero(n_fc));
    Eigen::MatrixXd Vf = (poly_1.vrep().first).transpose();
    // std::cout<<"U->V"<<std::endl;
    // poly_1.printHrep();
    // poly_1.printVrep();
    // myUtils::pretty_print(Vf, std::cout, "Vf");

    // A*V -> Uav
    Eigen::MatrixXd Vst =  (ASequence_[cid]*Vf);
    bool b_poly_Vst = poly_2.setVrep( Vst.transpose(), Eigen::VectorXd::Zero(Vst.cols()) );
    UavSequence_[cid] = poly_2.hrep().first;
    // std::cout<<"AV->U_(AV)"<<std::endl;    
    // poly_2.printVrep();
    // poly_2.printHrep();
    // myUtils::pretty_print(UavSequence_[cid], std::cout, "UavSequence_[cid]");
    // std::cout<<"======================================="<<std::endl;
    // Eigen::MatrixXd UavA = UavSequence_[cid]*ASequence_[cid];
    // myUtils::pretty_print(UavA, std::cout, "Uav*A");
  }  
}

void BeizierPlanner::buildInequalities(){
  // initialize
  H_ = Eigen::MatrixXd::Zero(0,0);
  h_ = Eigen::VectorXd::Zero(0);

  double alpha = mPlanParam->robotMass_/mPlanParam->timeHorizon_/mPlanParam->timeHorizon_;
  // std::cout<<"alpha = "<< alpha<<std::endl;

  // for(int cid(0); cid<3; cid++) {
    int cid = 1;{
    for( auto  &ycoeff : PwsSequence_[cid].by_.coeffs_){
      H_ = beizier_utils::vStack( H_, UavSequence_[cid] * ycoeff*alpha );
    }
    for( auto  &scoeff : PwsSequence_[cid].bs_.coeffs_){      
      h_ = beizier_utils::vStack( h_, UavSequence_[cid] * (- scoeff*alpha + ASequence_[cid]* FmSequence_[cid] ));
      // std::cout<<"scoeff*alpha = "<< scoeff*alpha<<std::endl;
    }
    // std::cout<<"ASequence_[cid]* FmSequence_[cid] = "<<ASequence_[cid]* FmSequence_[cid]<<std::endl;
  }

  // std::cout<<"H_ size = "<<H_.rows() << ", "<<H_.cols()<<std::endl;
  // std::cout<<"h_ size = "<<h_.size() <<std::endl;
}

void BeizierPlanner::solveQP(){

  Eigen::MatrixXd _H, _Aieq;
  Eigen::VectorXd _f, _bieq, y;
  // find y
  // 0.5(y-cmid)'(y-cmid) = 0.5y'y - cmid'y
  // such that H_y <= h_

  _H = Eigen::MatrixXd::Identity(3,3);
  _f = -0.5*(mPlanParam->comStart_ + mPlanParam->comGoal_);


  qp_solver_->setProblem(_H,_f,H_,h_);
  qp_solver_->solveProblem(y);

  std::cout<<" cmid =" << -_f.transpose() << std::endl;
  std::cout<<" y=" << y.transpose() << std::endl;

}


void BeizierPlanner::test(){

  Eigen::Vector3d y = 0.5*(mPlanParam->comStart_ + mPlanParam->comGoal_);
  // Eigen::Vector3d y = 0.3*mPlanParam->comStart_ + 0.7*mPlanParam->comGoal_;
  // Eigen::Vector3d y = mPlanParam->comStart_;
  // Eigen::Vector3d y = Eigen::Vector3d::Zero();
  
  
  Eigen::VectorXd Hy = H_*y;
  for(int i(0); i<h_.size(); ++i){
    // std::cout<<" Hy= "<<Hy[i]<< " < " << h_[i] << std::endl;    
    if(Hy[i] > h_[i])
      std::cout<<" Hy= "<<Hy[i]<< " > " << h_[i] << std::endl;    
  }

  Eigen::MatrixXd _H, _Aieq, _Aeq;
  Eigen::VectorXd _f, _bieq, _beq, Fc;

  Eigen::VectorXd w, htmp;
  double alpha = mPlanParam->robotMass_/mPlanParam->timeHorizon_/mPlanParam->timeHorizon_;
  double t=0.;
  int cid = 1;{
  // for(int cid(0); cid<3; cid++){
    t=0.;


    // check if Fc s.t. AFc = w-AFm , UFc<0?
    
    int n = FmSequence_[cid].size();
    _H = Eigen::MatrixXd::Identity(n,n);
    _f = Eigen::VectorXd::Zero(n);
    _Aeq = ASequence_[cid];
    
    _Aieq = FricConeSequence_[cid];
    _bieq = Eigen::VectorXd::Zero(FricConeSequence_[cid].rows());

    std::cout.precision(4);
    while(t<1.0){
      w = alpha* PwsSequence_[cid].getW(t,y);
      std::cout<<t<<" w-afm = " << (w - ASequence_[cid]* FmSequence_[cid]).transpose()<< std::endl;
      htmp = UavSequence_[cid]* (w - ASequence_[cid]* FmSequence_[cid]);
      

      for(int i(0); i<htmp.size(); ++i){
        if(htmp[i] > 0.0)
          std::cout<<t<<" htmp["<<i<< "]=" << htmp[i] << std::endl;
      }      

      _beq = (w - ASequence_[cid]* FmSequence_[cid]);    
      qp_solver_->setProblem(_H,_f,_Aieq,_bieq,_Aeq,_beq);
      qp_solver_->solveProblem(Fc);
      std::cout<<t<<" Fc["<<t<< "]=" << Fc.transpose() << std::endl;

      t = t+0.1;
    }
  }


  
}