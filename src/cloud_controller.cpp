/*******************************************************
 * Controller-in-Cloud: Cloud Side
 * 
 * Description:
 * source code to run an impedance controller with
 * optional force control as edge cloud.
 * 
 * Author: 
 * Fabian Jakob
 * Munich Institure of Robotics and System Intelligence
 * Technical University of Munich
 * fabian.jakob@tum.de
 * 
********************************************************/

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <regex>
#include <thread>
#include <iomanip>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 

// Custom library
#include "franka_kinematics.h"
#include "franka_dynamics.h"
#include "TDPA.h"
#include "print_utils.cpp"
#include "recorder.cpp"
#include "udp_utils.cpp"

// Timer
#include <time.h>
#include <ctime>

// Json related, parameter configuration
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

// Datatypes
#include "data_types.h"

using namespace Eigen;


void cloud_controller(json &parameter, bool &running, Data2Send &data_send, Data2Recv &data_recv, Data2Print &data_print, double &time_robot);


int main(int argc, char **argv) {

    /* Read and parse JSON file parameters */
    std::ifstream parameter_file("../config/cloud_parameter.json");
    json parameter = json::parse(parameter_file);

    /* Run parameters */
    bool running = true;
    double time_robot = 0.0;

    /* Print variables */
    Data2Print data_print;

    /* UDP variables */
    Data2Send data_send;
    Data2Recv data_recv;
    
    // Start threads;
    std::thread thread_send;
    std::thread thread_recv;
    std::thread thread_print;
    std::thread thread_cloud;

    thread_send  = std::thread(udp_send, std::ref(parameter), std::ref(running), std::ref(data_send));
    thread_recv  = std::thread(udp_recv, std::ref(parameter), std::ref(running), std::ref(data_recv));
    thread_print = std::thread(print_data, std::ref(parameter), std::ref(running), std::ref(data_print), std::ref(time_robot));
    thread_cloud = std::thread(cloud_controller, std::ref(parameter), std::ref(running), std::ref(data_send), std::ref(data_recv), std::ref(data_print), std::ref(time_robot));
 
    thread_cloud.join();
    running=false;

    thread_print.detach();
    thread_send.detach();
    thread_recv.detach();

    return 0;
}


void cloud_controller(json &parameter, bool &running, Data2Send &data_send, Data2Recv &data_recv, Data2Print &data_print, double &time_robot) {

    /* Run parameter */
    const double run_time = parameter["run_time"];
    double time_cloud = 0.0;
    double time_cloud_0 = 0.0; 
    double time_cloud_prev = 0.0;


    /* Recorder */
    int index = 0;
    double t_rec = run_time;
    t_rec = run_time;
    double t_sample = parameter["sample_time_init"];
    const int n_data_record = parameter["n_data_record"];
    Recorder recorder(t_rec, t_sample, n_data_record, "cloud_data", "../data/");
    std::mutex rec_mutex;


    /* Init Control Variables */
    // all arrays
    std::array<double, 7> q_cloud_array{0, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> dq_cloud_array = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> q_robot_array = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> dq_robot_array = {0, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> tau_com_cloud = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> tau_ext_array = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 6> f_ext_meas_array = {{0, 0, 0, 0, 0, 0}};
    
    Eigen::Matrix<double, 7, 1> q_init;
    Eigen::Matrix<double, 7, 1> q_cloud;
    Eigen::Matrix<double, 7, 1> dq_cloud;
    dq_cloud.setZero();

    // for trajectory
    Eigen::Vector9d desired_traj;
    Eigen::Vector3d p_init;
    Eigen::Vector3d p_des;
    Eigen::Matrix<double, 6, 1> error;
    Eigen::Matrix<double, 6, 1> dx_des, ddx_des;
    p_des.setZero();
    dx_des.setZero();
    ddx_des.setZero();
    
    // for force control
    bool force_control_active = parameter["force_control"];
    if (force_control_active) enable_force = 1.0;
    Eigen::VectorXd tau_ext(7);
    Eigen::VectorXd tau_ext_bias(7);
    Eigen::VectorXd tau_ext_filtered(7); 
    Eigen::VectorXd f_ext_meas(6);   
    Eigen::VectorXd f_ext_meas_bias(6);    
    Eigen::VectorXd f_ext_meas_filtered(6);      
    Eigen::VectorXd f_error_integral(6); 
    Eigen::VectorXd f_ext_kathib(6);    
    f_error_integral.setZero();
    tau_ext_filtered.setZero();
    f_ext_meas_filtered.setZero();

    Eigen::VectorXd tau_com(7);
    Eigen::VectorXd f_des(6), f_imp(6), f_force(6);
    Eigen::Matrix<double, 7, 7> Nullspace;
    Eigen::Matrix<double, 7, 6> M_inv_J_T;
    Eigen::Matrix<double, 7, 7> Mass;
    Eigen::Matrix<double, 6, 6> Mass_C;
    f_des.setZero();

    // kinematic
    Eigen::Matrix4d T_EE_cloud;
    Eigen::Matrix4d T_EE_robot;
    Eigen::Matrix<double, 6, 7> jacobian_cloud;
    Eigen::Matrix<double, 6, 7> jacobian_robot;
    Eigen::Vector3d position_cloud;
    Eigen::Vector3d position_robot;
    Eigen::Quaterniond orientation_init;
    Eigen::Quaterniond orientation_cloud;


    /* Dynamic Model */
    Eigen::VectorXd Xb(59);
    Xb << -0.0055114, 0, 0, 1.0691, -0.0066434, -0.0048947, -0.033545, 1.0332, 0.0021581, -3.2541, -0.027106, -0.0097099, -0.012624, -0.020563, 0.13049, 0.70071, 0.0069861, 0.6068, 0.18252, -0.006054, 0.010729, 0.69649, -0.53108, 1.8279, 0.021626, 0.004939, -0.009152, 0.0025572, 0.016561, -0.0039843, 0.083982, 0.0025833, 0.01349, -0.0029707, -0.0087703, 0.017069, 0.17619, -0.095175, 0.00049905, -0.0096179, 0.0026309, 0.0020262, -0.0060253, 0.006564, 0.0026488, 0.0063594, -0.018875, 0.041197, 0.11279, 0.09556, 0.055479, 0.0039106, 0.48431, 0.30568, 0.21134, 0.25618, 0.33338, 0.15525, 0.12302;
    Dynamics dyn(M_PI, 0.0, Xb);


    /* TDPA */
    bool TDPA_active = parameter["TDPA_active"];

    const int DoF = 7;
    TDPA<DoF> tdpa_cloud;
    
    std::array<double, 7> v_tdpa = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> f_tdpa = {{0, 0, 0, 0, 0, 0, 0}};

    std::array<double, 7> beta = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> E_diss = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> E_cloud_in = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> E_cloud_out = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> E_robot_in = {{0, 0, 0, 0, 0, 0, 0}};

    // position drift compensation
    double K_drift = parameter["k_pos_drift"];
    std::array<double, 7> position_error = {{0, 0, 0, 0, 0, 0, 0}};

    // passivity shortage augmentation
    double dissipation = 0.0;
    double shortage = 0.0;
    double eta = parameter["eta_passivity_shortage"];


    /* Data synchronization */
    bool first_data = true;
    double has_data_robot = 0;
    std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));

    /* Read Impedance and Force Control Parameters */
    double translational_stiffness = parameter["translational_stiffness"];
    double rotational_stiffness = parameter["rotational_stiffness"];
    double k_p = parameter["k_p"];
    double k_i = parameter["k_i"];

    // Impedance control
    Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
    stiffness.setZero();
    damping.setZero();
    stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
    damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);

    if (force_control_active) stiffness(2, 2) = 0;  // stifness zero for z axis (as z is force controlled, otherwise energy tank is needed)

    // Force control
    double desired_mass = 0.0;
    constexpr double target_mass = 1; 

    constexpr double mass_filter = 0.001; // filter for force trajectory 
    constexpr double force_filter = 0.3;  // filter for torque/force filtering

    // Timer init
    struct timespec t_start;
    struct timespec t_end;

    /*** CONTROL CALLBACK IN THE CLOUD ***/
    while (time_cloud <= run_time - 0.0011) {

        auto start = std::chrono::high_resolution_clock().now();
        clock_gettime(CLOCK_MONOTONIC, &t_start);

        /* Receive Data from Robot */
        if (data_recv.mutex.try_lock()) {
            time_robot = data_recv.time_robot;
            q_robot_array = data_recv.q_robot;
            dq_robot_array = data_recv.dq_robot;
            tau_ext_array = data_recv.tau_ext_robot;
            f_ext_meas_array = data_recv.f_ext_robot;
            E_robot_in = data_recv.E_robot_in;
            has_data_robot = data_recv.has_data_robot;

            data_recv.mutex.unlock();
        } else {
            std::cout << "No receive lock in controller thread" << std::endl;
        }

        if (first_data && (has_data_robot < 0.001))  {
            // do nothing unless robot receives first data
            continue;

        } else if (first_data && (has_data_robot > 0.001)) {
            /* Initialization */

            time_cloud_0 = time_robot;

            // Init Joints
            q_cloud_array = q_robot_array;
            q_init = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q_robot_array.data());

            // init robot kinematics
            T_EE_robot = Kinematics::ForwardKinematics(q_robot_array.data());
            p_init = T_EE_robot.block<3, 1>(0, 3);  

            // Init torques/forces
            tau_ext_bias = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(tau_ext_array.data());;
            f_ext_meas_bias = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(f_ext_meas_array.data());

            // Init orientation
            orientation_init = T_EE_robot.block<3, 3>(0, 0);

            first_data = false;

            time_cloud -= time_cloud_0;
        }

        time_cloud_prev = time_cloud;
        time_cloud += t_sample;
        

        /* Get kinematic measures from measured and integrated joint angles */
        q_cloud = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q_cloud_array.data());

        jacobian_cloud = Kinematics::ComputeJacobian(q_cloud_array.data());
        jacobian_robot = Kinematics::ComputeJacobian(q_robot_array.data());

        T_EE_cloud = Kinematics::ForwardKinematics(q_cloud_array.data());
        T_EE_robot = Kinematics::ForwardKinematics(q_robot_array.data());

        position_cloud = T_EE_cloud.block<3, 1>(0, 3);
        position_robot = T_EE_robot.block<3, 1>(0, 3);

        orientation_cloud = T_EE_cloud.block<3, 3>(0, 0);


        /* Obtain measured torques/forces */
        tau_ext = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(tau_ext_array.data());
        f_ext_meas = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(f_ext_meas_array.data());

        // offset correction
        tau_ext = tau_ext - tau_ext_bias;
        f_ext_meas = f_ext_meas - f_ext_meas_bias;

        // filtering
        tau_ext_filtered = force_filter * tau_ext_filtered + (1 - force_filter) * tau_ext;
        f_ext_meas_filtered = force_filter * f_ext_meas_filtered + (1 - force_filter) * f_ext_meas;
        

        /* TDPA */
        if (TDPA_active && !first_data) {
            /*
            * Controller is passive w.r.t. (-dq,tau)
            * => TDPA has to be passive w.r.t (dq,tau)
            */  

            // position drift compensation
            for (int i = 0; i < 7; i++) v_tdpa[i] = dq_robot_array[i] - K_drift * position_error[i];

            // shortage computation as integral of robot energy dissipation
            dissipation = dq_cloud.transpose() * jacobian_cloud.transpose() * damping *  jacobian_cloud * dq_cloud;
            shortage += t_sample*eta*dissipation; 

            tdpa_cloud.PassivityObserver(f_tdpa, v_tdpa, E_robot_in[0] + shortage, t_sample); 
            tdpa_cloud.PassivityController(f_tdpa, v_tdpa, t_sample); 

            E_cloud_in[0] = tdpa_cloud.GetInputEnergy();
            E_cloud_out[0] = tdpa_cloud.GetOutputEnergy();
            E_diss[0] = tdpa_cloud.GetDissipatedEnergy();
            beta[0] = tdpa_cloud.GetDamper();

            dq_cloud = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(v_tdpa.data());

        } else {
            dq_cloud = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(dq_robot_array.data());           
        }
        
        for (int i = 0; i < 7; i++) {
            q_cloud_array[i] = q_cloud_array[i] + t_sample * dq_cloud[i];
            dq_cloud_array[i] = dq_cloud[i];

            for (int i = 0; i < 7; i++) position_error[i] = q_cloud_array[i] - q_robot_array[i];
        }
        
        /* Compute desired trajectory */
        Kinematics::trajectory(time_cloud, p_init, desired_traj);

        p_des[0] = desired_traj[0];
        p_des[1] = desired_traj[1];
        p_des[2] = desired_traj[2];

        dx_des[0] = desired_traj[3];
        dx_des[1] = desired_traj[4];
        dx_des[2] = desired_traj[5];

        ddx_des[0] = desired_traj[6];
        ddx_des[1] = desired_traj[7];
        ddx_des[2] = desired_traj[8];

        /* Compute Impedance Control Error */
        // position error
        error.head(3) << position_cloud - p_des;
        std::cout << error.head(3) << "\n" << std::endl;

        // Orientation Error
        if (orientation_init.coeffs().dot(orientation_cloud.coeffs()) < 0.0) {
            orientation_cloud.coeffs() << -orientation_cloud.coeffs();
        }
        Eigen::Quaterniond error_quaternion(orientation_cloud.inverse() * orientation_init);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        error.tail(3) << -T_EE_cloud.block<3, 3>(0, 0) * error.tail(3);


        /* Compute Mass matrices and cartesian mappings */
        Mass = dyn.get_M(q_cloud);
        M_inv_J_T = Mass.inverse() * jacobian_cloud.transpose();
        Mass_C = (jacobian_cloud * M_inv_J_T).inverse();
        Nullspace = Eigen::MatrixXd::Identity(7, 7) - M_inv_J_T * Mass_C * jacobian_cloud;


        /* Map External Torque to Cartesian Force */
        f_ext_kathib = (M_inv_J_T * Mass_C).transpose() * tau_ext;


        /* Smooth Force Trajectory */ 
        desired_mass = mass_filter * target_mass + (1 - mass_filter) * desired_mass;
        f_des[2] = desired_mass * -9.81;


        /* Impedance Control law */
        f_imp = Mass_C * ddx_des - stiffness * error - damping * ((jacobian_cloud * dq_cloud) - dx_des);
        

        /* Admittance Control law */
        f_error_integral += t_sample * (f_des - f_ext_kathib);
        f_force = f_des + k_p * (f_des - f_ext_kathib) + k_i * f_error_integral;


        /* Mapping to Joint Torques */
        tau_com = jacobian_cloud.transpose() * (1.0*f_imp + enable_force*f_force) 
                        + 0.0*Nullspace.transpose()*(q_init - q_cloud); // Nullspace


        if (time_cloud >= run_time - 0.01) {
            // send ultimate zero torque to avoid danger
            tau_com.setZero();
        }

        /* Map Eigen::Matrix to array */
        Eigen::VectorXd::Map(&tau_com_cloud[0], 7) = tau_com;
        Eigen::VectorXd::Map(&f_tdpa[0], 7) = tau_com;

        std::array<double, 6> f_ext2_array{};
        Eigen::VectorXd::Map(&f_ext2_array[0], 6) = f_ext_kathib;

        std::array<double, 6> f_imp_array{};
        Eigen::VectorXd::Map(&f_imp_array[0], 6) = f_imp;

        std::array<double, 6> error_array{};
        Eigen::VectorXd::Map(&error_array[0], 6) = error;

        std::array<double, 3> velocity_cloud{};
        Eigen::VectorXd::Map(&velocity_cloud[0], 3) = jacobian_cloud.block<3,7>(0,0) * dq_cloud;

        std::array<double, 3> velocity_robot{};
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_robot(dq_robot_array.data());
        Eigen::VectorXd::Map(&velocity_robot[0], 3) = jacobian_robot.block<3,7>(0,0) * dq_robot;

        /* Send Torque Commands to Robot */
        {
            std::lock_guard<std::mutex> lck_send(data_send.mutex);

            data_send.time_cloud = time_cloud;
            data_send.q_cloud = q_cloud_array;
            data_send.dq_cloud = dq_cloud_array;
            data_send.tau_com_cloud = tau_com_cloud;
            data_send.f_com_cloud = f_imp_array;
            data_send.E_cloud_in = E_cloud_in;
            data_send.has_data_cloud = 1;

            send_allowed = true;
        }

        /* Send data to print */
        if (data_print.mutex.try_lock()) {
            data_print.has_data = true;
            data_print.time_robot = time_robot;
            data_print.q = q_robot_array;
            data_print.dq = dq_robot_array;
            data_print.tau_ext = tau_ext_array;
            data_print.f_ext = f_ext_meas_array;
            data_print.f_ext2 = f_ext2_array;
            data_print.err = error_array;
            data_print.period = time_cloud - time_cloud_prev;
            data_print.mutex.unlock();
        }

        /* Recording */
        if (time_cloud < t_rec) {
            recorder.addToRec(index);
            recorder.addToRec(time_cloud);
            recorder.addToRec(time_robot);
            recorder.addToRec(q_robot_array);   // received robot position
            recorder.addToRec(dq_robot_array);  // received robot velocity
            recorder.addToRec(q_cloud_array); // desired velocity after PC
            recorder.addToRec(dq_cloud_array);  // integrated PC velocity
            recorder.addToRec(tau_ext_array);   // received robot external torque
            recorder.addToRec(f_ext_meas); // received robot external force
            recorder.addToRec(f_ext_meas_filtered); // received robot external force
            recorder.addToRec(f_ext_kathib); // received robot external force
            recorder.addToRec(f_des[2]);
            recorder.addToRec(error_array);
            recorder.addToRec(p_des);
            recorder.addToRec(dx_des);
            recorder.addToRec(position_cloud);
            recorder.addToRec(position_robot);
            recorder.addToRec(velocity_cloud);
            recorder.addToRec(velocity_robot);
            recorder.addToRec(E_cloud_in[0]);
            recorder.addToRec(E_cloud_out[0]);
            recorder.addToRec(E_robot_in[0]);
            recorder.addToRec(E_diss[0]);      
            recorder.addToRec(beta[0]);        
            recorder.addToRec(tau_com_cloud);  // calculated command torque in cloud
            recorder.next();
            index++;
        }

        cv_send.notify_one();

        // Loop synchronization
        auto end = std::chrono::high_resolution_clock().now();
        std::chrono::duration<double, std::nano> elapsed = end - start;
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000) - elapsed);

        clock_gettime(CLOCK_MONOTONIC, &t_end);
        t_sample = static_cast<double>((static_cast<int>(t_end.tv_nsec) - static_cast<int>(t_start.tv_nsec))) / 1000000000.0;
        // counteract number overflow
        if (t_sample < 0.001) {
            t_sample = 0.001;
        }
    }

    send_allowed = true;
    cv_send.notify_one();

}
