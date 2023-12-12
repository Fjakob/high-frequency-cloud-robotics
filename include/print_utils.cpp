#include "print_utils.h"


void print_data(json parameter, bool &running, Data2Print &data_print, double &time_local) {

    double print_rate = parameter["print_rate"];

    while (running) {
        // Sleep to achieve the desired print rate.
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));

        // Try to lock data to avoid read write collisions.
        if (data_print.mutex.try_lock()) {
            if (data_print.has_data) {
                std::array<double, 7> tau_error{};
                double error_rms(0.0);
                std::array<double, 7> tau_d_actual{};
                for (size_t i = 0; i < 7; ++i) {
                    tau_d_actual[i] = data_print.tau_d_last[i] + data_print.gravity[i];
                    tau_error[i] = tau_d_actual[i] - data_print.robot_state.tau_J[i];
                    error_rms += std::pow(tau_error[i], 2.0) / tau_error.size();
                }
                error_rms = std::sqrt(error_rms);

                // Print data to console
                std::cout << "Time [s]: " << time_local << std::endl
                          << "tau_error [Nm]: " << tau_error << std::endl
                          << "dq [Nm]: " << data_print.robot_state.dq << std::endl
                          << "q_measured [Rad]: " << data_print.qmes << std::endl
                          << "tau_com_cloud =" << data_print.tau_com_cloud << std::endl
                          << "qdes_F =" << data_print.q_cloudF << std::endl
                          << "f_ext =" << data_print.f_ext << std::endl

                          << "root mean square of tau_error [Nm]: " << error_rms << std::endl
                          << "-----------------------" << std::endl;
                data_print.has_data = false;
            }
            data_print.mutex.unlock();
        }
    }
}