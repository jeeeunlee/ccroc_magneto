#pragma once
#pragma GCC diagnostic ignored "-Wreorder"

#include <eigen3/Eigen/Eigen>
#include <array>
#include <iostream>
#include <memory>
#include <vector>
#include "beizier_planner/RobotModel.hpp"

class ContactState {
   public:
    ContactState()
        : timeIni(0.),
          timeEnd(0.),
          contactType(0),
          position(Eigen::Vector3d::Zero()),
          orientation(Eigen::Quaternion<double>::Identity()),
          fricCoeff(1.0),
          frcMag(Eigen::Vector3d::Zero()){

          };
    ContactState(const Eigen::VectorXd& cs){
        // cs: VectorXd(14) represent contact state
        timeIni = cs(0);
        timeEnd = cs(1);        
        position = cs.segment<3>(2);
        orientation = Eigen::Quaternion<double>(cs[5], cs[6], cs[7], cs[8]);
        contactType = (static_cast<int>(cs(9)));
        fricCoeff = cs(10);
        frcMag = Eigen::Vector3d::Zero();
        frcMag(2) = cs(11);
    };
    virtual ~ContactState(){};

    std::string toString() const {
        std::stringstream text;
        text << "    time         " << timeIni << " ~ " << timeEnd << "\n";
        text << "    contact type " << static_cast<int>(contactType) << "\n";
        text << "    position     " << position.transpose() << "\n";
        text << "    orientation  " << "\n";
        text << orientation.toRotationMatrix() << "\n";
        return text.str();
    };
    friend std::ostream& operator<<(std::ostream& os, const ContactState& obj) {
        return os << obj.toString();
    }

    Eigen::Vector3d position;
    int contactType;
    double timeIni, timeEnd;
    Eigen::Quaternion<double> orientation;
    // added for magneto
    double fricCoeff;
    Eigen::Vector3d frcMag;
};

class ContactStateSequence {
   public:
    ContactStateSequence(){};
    virtual ~ContactStateSequence(){};

    std::string toString() const {
        std::stringstream text;
        for (int eff_id = 0; eff_id < RobotModel::numEEf; eff_id++) {
            text << "eff_id " << eff_id << "\n";
            for (int cnt_id = 0; cnt_id < eEfContacts[eff_id].size();
                 cnt_id++) {
                text << "  cnt_id " << cnt_id << "\n";
                text << eEfContacts[eff_id][cnt_id];
            }
        }
        return text.str();
    };
    friend std::ostream& operator<<(std::ostream& os,
                                    const ContactStateSequence& obj) {
        return os << obj.toString();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::array<std::vector<ContactState>, RobotModel::numEEf> eEfContacts;
};
