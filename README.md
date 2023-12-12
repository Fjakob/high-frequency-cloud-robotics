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

1. [the robot side](https://github.com/Fjakob/high-frequency-cloud-robotics)
2. [the cloud side](https://github.com/Fjakob/high-frequency-cloud-robotics/tree/controller-cloud)

Make sure to clone the respective branch on one PC connected to the robot and one PC running the cloud. Both PC has to be located in the same network. For communication, a [UDP socket](./include/udp_utils.cpp) is implemented. This branch runs the **cloud** side, and implements the controller therefore. Make sure, to adjust the control gains wisely to not risk instability or discontinuities. Note, that stability might also depend on the initial configuration of the robot. Note, that the model parameters used for the dynamic model are individually identified and might now apply to your specific Franka Robot.


## Run the Code

The code has been written and tested on Ubuntu 20.04 and 22.04. A supported compiler, [cmake](https://cmake.org) and [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) is required, e.g. by running

```
sudo apt install build-essential
sudo apt install cmake
sudo apt install libeigen3-dev
```


### Build the Code

Clone the repo. To build the code, run

```
cmake -S . -B build
make -C build
```

### Edit the Configuration File

Edit the configuration file [cloud_parameter.json](./config/cloud_parameter.json) and adjust 

1. the IP of the robot PC
2. The run time
3. whether you want to use TDPA
4. if you use TDPA, the position drift compensation gain and the passivity augmentation constant `eta`
5. the control gains of the impedance controller
6. whether you want to use force control and the force control PI gains

The ports can be kept unless your machine is using them already. Optionally adjust the prompt printing rate and the recorder setting, that will save all data afterwards.


### Run the Executable

If your configurations are set, run the cloud controller with

``` 
./build/cloud_controller
```

When the cloud controller is running, you can start the local controller. The code will terminate after the run time, specified by the configuration file. 

Try running the setup using different configurations and try to impose artificial delay using [tcgui](https://github.com/tum-lkn/tcgui).


## Visualization

After terminating the code, relevant variables will be saves in a Matlab file and saved in the directory [data](./data/). For visualization, a [Matlab script](./data/plot_cloud_data.m) that parses and plots the data is provided.

<p align="center">
  <img src="https://github.com/Fjakob/high-frequency-cloud-robotics/assets/78848571/f267f0ad-88be-4161-9f03-eba0fb5788f7" width="500"/>
</p>

