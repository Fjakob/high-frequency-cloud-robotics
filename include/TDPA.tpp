#pragma once


template <int DOF>
TDPA<DOF>::TDPA() {
    is_passive_ = false;
    E_in_ = 0;
    E_out_ = 0;
    E_diss_ = 0;
    E_PC_ = 0;
    squared_flow_norm_ = 0;
    damper_ = 0;
}


template <int DOF>
inline void TDPA<DOF>::innerProduct(std::array<double, DOF> vec1, std::array<double, DOF> vec2, double &result) {

    result = 0;
    for (int i = 0; i < DOF; i++) {
        result += vec1[i] * vec2[i];
    }

}


template <int DOF>
void TDPA<DOF>::PassivityObserver(std::array<double, DOF> flow, std::array<double, DOF> effort, double E_in_remote, double deltaT) {

    // Compute power
    double power;
    innerProduct(flow, effort, power);

    // Update Energies
    if (power > 0) {
        E_in_ += deltaT * power;
        
    } else {
        E_out_ -= deltaT * power;
    }

    // compute squared norm of flow
    innerProduct(flow, flow, squared_flow_norm_);

    // compute available energy
    E_PC_ = E_in_remote - E_out_ + E_diss_;

    if (E_PC_ >= 0) {
        is_passive_ = true;
    } else {
        is_passive_ = false;
    }
}


template <int DOF>
void TDPA<DOF>::PassivityController(std::array<double, DOF> &flow, std::array<double, DOF> &effort, double deltaT) {

    // damp effort, if communication channel is not passive
    if ( !is_passive_ && (squared_flow_norm_ > 1e-6) ) {
        damper_ = - E_PC_ / (deltaT * squared_flow_norm_);

        for (int i = 0; i < DOF; i++) {
            effort[i] = effort[i] + damper_ * flow[i];
        }

        E_diss_ += deltaT * damper_ * squared_flow_norm_;

        // std::cout << "TDPA active with beta=" << damper_ << std::endl;

    } else {
        damper_ = 0;

    }
}
