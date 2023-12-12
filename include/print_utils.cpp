#include "print_utils.h"

void print_data(json parameter, bool &running, Data2Print &data_print, double &time_robot) {
    
    double print_rate = parameter["print_rate"];

    while (running) {
        // Sleep to achieve the desired print rate.
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));

        // Try to lock data to avoid read write collisions.
        if (data_print.mutex.try_lock()) {
            if (data_print.has_data) {
                // Print data to console
                std::cout << "Time [s]: " << data_print.time_robot << std::endl
                          << "period: " << data_print.period << std::endl
                          << "q [Rad]: " << data_print.q << std::endl
                          << "dq [Rad]: " << data_print.dq << std::endl
                          << "tau_ext =" << data_print.tau_ext << std::endl
                          << "f_ext =" << data_print.f_ext << std::endl
                          << "f_ext2 =" << data_print.f_ext2 << std::endl
                          << "err =" << data_print.err << std::endl
                          << "-----------------------" << std::endl;
                data_print.has_data = false;
            }
            data_print.mutex.unlock();
        }
    }
}