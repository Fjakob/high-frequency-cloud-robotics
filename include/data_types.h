#pragma once

#include <mutex>
#include <array>
#include <franka/robot_state.h>


struct Data2Send {
    std::mutex mutex;
    double time_local = 0;
    std::array<double, 7> q_robot{0, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> dq_robot = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> tau_ext_robot = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 6> f_ext_robot = {0, 0, 0, 0, 0, 0};
    std::array<double, 7> E_robot_in = {0, 0, 0, 0, 0, 0, 0};
    double has_data_robot = 0;
};


struct Data2Recv {
    std::mutex mutex;
    double time_remote = 0;
    std::array<double, 7> q_cloud = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> dq_cloud = {0, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> tau_com_cloud = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 6> f_com_cloud = {0, 0, 0, 0, 0, 0};
    std::array<double, 7> E_cloud_in = {0, 0, 0, 0, 0, 0, 0};
    double has_data_cloud = 0;
};


struct Data2Print {
    std::mutex mutex;
    bool has_data;
    double time_local;
    std::array<double, 7> tau_d_last;
    franka::RobotState robot_state;
    std::array<double, 7> gravity;
    std::array<double, 7> qmes;
    std::array<double, 7> t_ext;
    std::array<double, 7> q_cloudF;
    std::array<double, 7> q_cloud;
    std::array<double, 6> f_ext;
    std::array<double, 7> tau_com_cloud;
};