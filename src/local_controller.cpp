/*******************************************************
 * Controller-in-Cloud: Local Side
 * 
 * Description:
 * source code to connect to Franka Robotics robot and
 * apply control commands received by an edge-cloud in
 * real-time.
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
#include <cmath>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <regex>
#include <thread>

// libfranka
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

// custom library
#include "motion_generator.cpp"
#include "recorder.cpp"
#include "udp_utils.cpp"
#include "print_utils.cpp"
#include "TDPA.h"

// json import
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

// data types
#include "data_types.h"


void drive_robot(json &parameter, bool &running, Data2Send &data_send, Data2Recv &data_recv, Data2Print &data_print, double &time_local);


int main(int argc, char **argv) {

    /* Read and parse JSON file parameters */
    std::ifstream parameter_file("../config/local_parameter.json");
    json parameter = json::parse(parameter_file);

    /* Run parameters */
    bool running = true;
    double time_local = 0.0;

    /* Print variables */
    Data2Print data_print;

    /* UDP variables */
    Data2Send data_send;
    Data2Recv data_recv;
    
    // Start threads;
    std::thread thread_send;
    std::thread thread_recv;
    std::thread thread_print;
    std::thread thread_robot;

    thread_send  = std::thread(udp_send, std::ref(parameter), std::ref(running), std::ref(data_send));
    thread_recv  = std::thread(udp_recv, std::ref(parameter), std::ref(running), std::ref(data_recv));
    thread_print = std::thread(print_data, std::ref(parameter), std::ref(running), std::ref(data_print), std::ref(time_local));
    thread_robot = std::thread(drive_robot, std::ref(parameter), std::ref(running), std::ref(data_send), std::ref(data_recv), std::ref(data_print), std::ref(time_local));

    thread_robot.join();
    thread_print.join();
    thread_send.detach();
    thread_recv.detach();

    return 0;
}


void drive_robot(json &parameter, bool &running, Data2Send &data_send, Data2Recv &data_recv, Data2Print &data_print, double &time_local) {

    // Run parameters
    const double run_time = parameter["run_time"];
    double time_remote = 0.0;

    /* Recorder */
    int index = 0;
    double t_rec = run_time;
    double t_sample = parameter["sample_time_init"];
    const int n_data_record = parameter["n_data_record"];
    Recorder recorder(t_rec, t_sample, n_data_record, "robot_data", "../data/");
    std::mutex rec_mutex;

    /* Init control variables */
    std::array<double, 7> q_robot= {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> dq_robot = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> q_cloud = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> dq_cloud = {0, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> tau_ext_robot = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> tau_com_cloud = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> tau_com_cloudF = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 6> f_ext_robot = {{0, 0, 0, 0, 0, 0}};
    std::array<double, 6> f_com_cloud = {{0, 0, 0, 0, 0, 0}};
    std::array<double, 7> q_cloudF = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> dq_cloudF = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> tau_d_calculated;
    std::array<double, 7> tau_d_calculated_old = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> tau_d_rate_limited;

    /* TDPA for delay stabilization */
    bool TDPA_active = parameter["TDPA_active"];

    const int DoF = 7;

    TDPA<DoF> tdpa_local;
    
    std::array<double, 7> v_tdpa = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> f_tdpa = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> E_cloud_in = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> E_robot_in = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> E_robot_out = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> E_diss = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> alpha = {{0, 0, 0, 0, 0, 0, 0}};

    /* Passivity shortage compensation */
    double mu_friction = 1.2;
    double dissipation = 0.0;
    double shortage = 0.0;
    double eta = parameter["eta_passivity_shortage"];


    try {
        /* Connect to robot */
        franka::Robot robot(parameter["robot_ip"]);
        robot.setCollisionBehavior(
            {{60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0}}, {{60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0}},
            {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}}, {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},
            {{60.0, 60.0, 60.0, 60.0, 60.0, 60.0}}, {{60.0, 60.0, 60.0, 60.0, 60.0, 20.0}},
            {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}}, {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}});
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

        franka::RobotState initial_state = robot.readOnce();

        /* Move to init pose */
        std::array<double, 7> q_init = parameter["q_init"];
        MotionGenerator motion_generator(0.5, q_init);
        std::cout << "WARNING: Press to move robot to initial pose. Make sure no obstacles are present." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration.\n" << std::endl;

        /* Load the kinematics and dynamics model. */ 
        franka::Model model = robot.loadModel();

        initial_state = robot.readOnce();
        Eigen::VectorXd initial_tau_ext(7), tau_error_integral(7);

        /* Read torque sensor offset */
        std::array<double, 7> gravity_array = model.gravity(initial_state);
        Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state.tau_J.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(gravity_array.data());
        initial_tau_ext = initial_tau_measured - initial_gravity;


        /* CONTROL CALLBACK FUNCTION */
        std::function<franka::Torques(const franka::RobotState &, franka::Duration)> local_control_callback = [&](const franka::RobotState &state, franka::Duration period) -> franka::Torques {
            
            if (period.toSec() == 0) {
                period = franka::Duration(1);
            } 
            time_local += period.toSec();

            /* Measure TCP */
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(state.O_T_EE.data()));
            Eigen::Vector3d position(transform.translation());

            /* Measure external torques/forces */
            tau_ext_robot = state.tau_ext_hat_filtered;
            f_ext_robot = state.O_F_ext_hat_K;

            /* Measure joint angles/velocities */
            q_robot = state.q;
            dq_robot = state.dq;

            /* Receive data from Cloud */
            if (data_recv.mutex.try_lock()) {
                time_remote = data_recv.time_remote;
                q_cloud = data_recv.q_cloud;
                dq_cloud = data_recv.dq_cloud;
                tau_com_cloud = data_recv.tau_com_cloud;
                f_com_cloud = data_recv.f_com_cloud;
                E_cloud_in = data_recv.E_cloud_in;

                data_recv.mutex.unlock();
            } else {
                std::cout << "No receive lock" << std::endl;
            }

            /* TDPA */
            if (TDPA_active) {
                /*
                Robot is passive w.r.t. (dq,tau) 
                => TDPA has to be passive w.r.t (-dq,tau)
                */ 
                for(int idx=0; idx<7; idx++) v_tdpa[idx] = -state.dq[idx];
                f_tdpa = tau_com_cloud;

                /* shortage computation as integral of robot energy dissipation */
                dissipation = 0;
                for(int idx=0; idx<7; idx++) dissipation += mu_friction*state.dq[idx]*state.dq[idx];
                shortage += period.toSec()*eta*dissipation;

                tdpa_local.PassivityObserver(v_tdpa, f_tdpa, E_cloud_in[0] + shortage, period.toSec());
                tdpa_local.PassivityController(v_tdpa, f_tdpa, period.toSec());

                E_robot_in[0] = tdpa_local.GetInputEnergy();
                E_robot_out[0] = tdpa_local.GetOutputEnergy();
                E_diss[0] = tdpa_local.GetDissipatedEnergy();
                alpha[0] = tdpa_local.GetDamper();

                tau_d_calculated = f_tdpa;
            } else {
                tau_d_calculated = tau_com_cloud;
            }

            /* Augment virtual friction */
            for(int idx=0; idx<7; idx++) tau_d_calculated[idx] = 0.9*tau_d_calculated_old[idx] + 0.1*(tau_d_calculated[idx] - mu_friction*state.dq[idx]);
            for(int idx=0; idx<7; idx++) tau_d_calculated_old[idx] = tau_d_calculated[idx];

            tau_d_rate_limited = franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

            /* Send data to cloud */
            {
                std::lock_guard<std::mutex> lck_send(data_send.mutex);

                data_send.time_local = time_local;
                data_send.q_robot = q_robot;
                data_send.dq_robot = dq_robot;
                data_send.tau_ext_robot = tau_ext_robot;
                data_send.f_ext_robot = f_ext_robot;
                data_send.E_robot_in = E_robot_in;
                data_send.has_data_robot = 1;

                send_allowed = true;
            } cv_send.notify_one();


            /* Update data for print thread */
            if (data_print.mutex.try_lock()) {
                data_print.has_data = true;
                data_print.time_local = time_local;
                data_print.robot_state = state;
                data_print.qmes = state.q;
                data_print.t_ext = tau_ext_robot;
                data_print.tau_d_last = tau_d_rate_limited;
                data_print.gravity = model.gravity(state);
                data_print.q_cloud = q_cloud;
                data_print.q_cloudF = q_cloudF;
                data_print.f_ext = f_ext_robot;
                data_print.tau_com_cloud = tau_com_cloud;
                data_print.mutex.unlock();
            }


            /* Log data by recorder */
            if (time_local < t_rec) {
                recorder.addToRec(index);
                recorder.addToRec(time_local);    // time
                recorder.addToRec(time_remote);   // time at cloud
                recorder.addToRec(q_robot);       // joint angles
                recorder.addToRec(q_cloud);       // delayed joint angles
                recorder.addToRec(dq_robot);      // joint velocities
                recorder.addToRec(dq_cloud);      // delayed joint velocities
                recorder.addToRec(tau_ext_robot); // measured external torque
                recorder.addToRec(f_ext_robot);   // measured external wrench
                recorder.addToRec(tau_com_cloud); // delayed received torque command
                recorder.addToRec(f_com_cloud);   // delayed received force command
                recorder.addToRec(E_robot_in[0]);    // TDPA
                recorder.addToRec(E_robot_out[0]);   // TDPA
                recorder.addToRec(E_cloud_in[0]);    // TDPA
                recorder.addToRec(E_diss[0]);        // TDPA
                recorder.addToRec(alpha[0]);         // TDPA
                recorder.addToRec(tau_d_calculated); // adjusted applied torques
                recorder.addToRec(position);      // endeffector position
                
                recorder.next();
                index++;
            }

            if (time_local >= run_time) {
                running = false;
                franka::Torques zerotorque = tau_d_rate_limited;
                std::cout << "*** Finished Motion ***" << std::endl;

                return franka::MotionFinished(zerotorque);
            }

            return tau_d_calculated;
        };

        /* Start real-time control loop */
        robot.control(local_control_callback);

    } catch (const franka::Exception &ex) {
        std::cerr << ex.what() << std::endl;

        std::cout << "Program stops running" << std::endl;
        running = false;
    }
}