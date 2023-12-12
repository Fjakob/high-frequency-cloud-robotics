# High-Frequency Cloud Robotics

## Background

This repo implements the paper (currently in submission)

> Fabian Jakob, Xiao Chen, Hamid Sadeghian, Sami Haddadin. 2024. Enhancing the Tracking Performance of Passivity-based High-Frequency
Robot Cloud Control.

The purpose of this repo is to provide an exemplary implementation of a framwork to outsource high-frequency robot computations to the edge-cloud. Stabilization w.r.t. to communication delay and package loss is handled by the [Time Domain Passivity Approach (TDPA)](./include/TDPA.tpp). To handle the performance tradeoff, a position drift compensation and passivity-excess augmentation is implemented. For further details, refer to the paper.

<p align="center">
  <img src="https://github.com/Fjakob/high-frequency-cloud-robotics/assets/78848571/2caf04bf-264b-463f-a264-935bf04f7213" width="500"/>
</p>

## Description

This repo is divided into two branches:

1. the robot side
2. the cloud side

Make sure to clone the respective branch on one PC connected to the robot and one PC running the cloud. Both PC has to be located in the same network. For communication, a [UDP socket](./include/udp_utils.cpp) is implemented. This branch runs the **robot** side, and is only compatible with [Franka Robotics](https://franka.de/) robots that have the Franka Control Interface (FCI) feature installed.


## Run the Code

The code has been written and tested on Ubuntu 20.04 and 22.04. A supported compiler and [cmake](https://cmake.org) is required, e.g. by running

```
sudo apt install build-essential
sudo apt install cmake
```

The dependencies are [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) and [libfranka](https://frankaemika.github.io/docs/libfranka.html). Make sure to clone and build libfranka into your home directory according to [the installation instructions](https://frankaemika.github.io/docs/installation_linux.html).


### Build the Code

Clone the repo. To build the code, run

```
cmake -S . -B build
make -C build
```

### Edit the Configuration File

Edit the configuration file [local_parameter.json](./config/local_parameter.json) and adjust 

1. your robot IP
2. the IP of the cloud PC
3. the initial joint configuration that the robot should move to
4. The run time
5. whether you want to use TDPA
6. if you use TDPA, the passivity augmentation constant `eta`

The ports can be kept unless your machine is using them already. Optionally adjust the prompt printing rate and the recorder setting, that will save all data afterwards.


### Run the Executable

If your configurations are set, start the cloud controller on the remote PC. If the cloud is ready, run the code with

``` 
./build/local_controller
```

Make sure that the robot FCI is enabled and in execution mode. The code will terminate after the run time, specified by the configuration file. 

Try running the setup using different configurations and try to impose artificial delay using [tcgui](https://github.com/tum-lkn/tcgui).


## Visualization

After terminating the code, relevant variables will be saves in a Matlab file and saved in the directory [data](./data/). For visualization, a [Matlab script](./data/plot_robot_data.m) that parses and plots the data is provided.

<p align="center">
  <img src="https://github.com/Fjakob/high-frequency-cloud-robotics/assets/78848571/f267f0ad-88be-4161-9f03-eba0fb5788f7" width="500"/>
</p>

