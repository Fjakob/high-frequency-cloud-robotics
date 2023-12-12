/********************************************************
 * Controller-in-Cloud: Time Domain Passivity Approach
 * 
 * Description:
 * TDPA library that performs communication passivation
 * 
 * Author: 
 * Fabian Jakob
 * Munich Institure of Robotics and System Intelligence
 * Technical University of Munich
 * fabian.jakob@tum.de
 * 
*********************************************************/

#pragma once

#include <stdlib.h>
#include <iostream>
#include <array>

template <int DOF> class TDPA {
   public:
    TDPA();

    void PassivityObserver(std::array<double, DOF> flow, std::array<double, DOF> effort, double E_in_remote, double deltaT);
    void PassivityController(std::array<double, DOF> &flow, std::array<double, DOF> &effort, double deltaT);

    bool is_passive() {return is_passive_; };
    double GetInputEnergy() { return E_in_; };
    double GetOutputEnergy() { return E_out_; };
    double GetDissipatedEnergy() { return E_diss_; };
    double GetDamper() { return damper_; };

   private:
    bool is_passive_;
    double E_in_;
    double E_out_;
    double E_diss_;
    double E_PC_;
    double squared_flow_norm_;
    double damper_;

    inline void innerProduct(std::array<double, DOF> vec1, std::array<double, DOF> vec2, double &result);
};

#include "TDPA.tpp"