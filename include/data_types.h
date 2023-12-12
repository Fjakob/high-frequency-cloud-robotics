/********************************************************
 * Controller-in-Cloud: Data types
 * 
 * Description:
 * Data types for print and UDP threads
 * 
*********************************************************/


#pragma once

#include <mutex>
#include <array>
#include <Eigen/Dense>


namespace Eigen {
    typedef Eigen::Matrix<double, 9, 1> Vector9d;
}


struct Data2Send {
    std::mutex mutex;
    double time_cloud = 0;
    std::array<double, 7> q_cloud = {0, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> dq_cloud = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> tau_com_cloud = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 6> f_com_cloud = {{0, 0, 0, 0, 0, 0}};
    std::array<double, 7> E_cloud_in = {{0, 0, 0, 0, 0, 0, 0}};
    double has_data_cloud = 0;
};


struct Data2Recv {
    std::mutex mutex;
    double time_robot = 0;
    std::array<double, 7> q_robot = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> dq_robot = {0, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> tau_ext_robot = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 6> f_ext_robot = {{0, 0, 0, 0, 0, 0}};
    std::array<double, 7> E_robot_in = {{0, 0, 0, 0, 0, 0, 0}};
    double has_data_robot = 0;
};


struct Data2Print {
    std::mutex mutex;
    bool has_data;
    double time_robot;
    double period;
    std::array<double, 7> q;
    std::array<double, 7> dq;
    std::array<double, 7> tau_ext;
    std::array<double, 7> t_ext;
    std::array<double, 6> f_ext;
    std::array<double, 6> f_ext2;
    std::array<double, 6> err;
};