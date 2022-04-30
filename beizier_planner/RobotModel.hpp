#pragma once

#include <stdexcept>
#include <stdio.h>
#include <string>

class RobotModel {
public:
  // Number of EndEffector
  static constexpr int numEEf = 4;

  // ID of EndEffector
  enum class EEfID { AL = 0, AR = 1, BL = 2, BR = 3 };

  // Function to map EndEffector Id to string
  static std::string eEfIdToString(int eefId_) {
    switch (eefId_) {
    case static_cast<int>(EEfID::AL): {
      return std::string("al");
      break;
    }
    case static_cast<int>(EEfID::AR): {
      return std::string("ar");
      break;
    }
    case static_cast<int>(EEfID::BL): {
      return std::string("bl");
      break;
    }
    case static_cast<int>(EEfID::BR): {
      return std::string("br");
      break;
    }
    default: {
      throw std::runtime_error(
          "End Effector ID not handled in effIdToString. \n");
    }
    }
  }
};
