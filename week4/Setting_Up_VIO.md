# Setting Up VIO (VINS-Fusion)

First, configure dependencies

Reference website

[Compile and Run VINS-Fusion on Ubuntu18.04_18.04vins-fusion Running - CSDN Blog](https://blog.csdn.net/xywy2008/article/details/122473298)

1. Configure ceres

Official website: [Installation — Ceres Solver (ceres-solver.org)](http://ceres-solver.org/installation.html)

For ubuntu18.04, it is recommended to download Ceres Solver 2.0.0

Go to

`https://ceres-solver.googlesource.com/ceres-solver`

Then find the tag 2.0.0 and click the tgz next to the commit number, which will download the compressed file ceres-solver-2.0.0.tar.gz

```jsx
mkdir -p ceres-solver-2.0.0

```

Then move the compressed file ceres-solver-2.0.0.tar.gz into the newly created folder and unzip it

```jsx
cd ceres-solver-2.0.0
tar zxf ceres-solver-2.0.0.tar.gz

```

Configure dependencies before compiling

```
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse (optional)
sudo apt-get install libsuitesparse-dev

```

Then compile

```jsx
cd ceres-solver-2.0.0
mkdir ceres-bin
cd ceres-bin
cmake ../
make
sudo make install

```

1. Configure vins-fusion

```
    cd ~/catkin_ws/src
    git clone <https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git>
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash

```

The next step to download the EuRoC resource package is optional, it is only to verify that vins-fusion can work normally

If you want to download it, click under ROS bag


1. Compile vins

```jsx
cp -r ~/XTDrone/sensing/slam/vio/VINS-Fusion ~/catkin_ws/src/
mkdir ~/catkin_ws/scripts/
cp ~/XTDrone/sensing/slam/vio/xtdrone_run_vio.sh ~/catkin_ws/scripts/
cd ~/catkin_ws
catkin_make

```

Here it may report an error due to insufficient swap space and memory, you can use catkin_make -j2 to limit the number of parallel compilations to limit memory usage

Flight control configuration

For version 1.1, you can directly modify

```jsx
gedit ~/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/rcS

```

For version 1.3, directly modify the compiled data

```jsx
gedit ~/PX4_Firmware/build/px4_sitl_default/etc/init.d-posix/rcS

```

Modify the parameters

The original code is

```
# GPS used
param set EKF2_AID_MASK 1
# Vision used and GPS denied
#param set EKF2_AID_MASK 24

# Barometer used for height measurement
param set EKF2_HGT_MODE 0
# Barometer denied and vision used for height measurement
#param set EKF2_HGT_MODE 3

```

After modification

```
# GPS used
#param set EKF2_AID_MASK 1
# Vision used and GPS denied
param set EKF2_AID_MASK 24

# Barometer used for height measurement
#param set EKF2_HGT_MODE 0
# Barometer denied and vision used for height measurement
param set EKF2_HGT_MODE 3

```

Here, the mode is switched by commenting.

1. Start the simulator

```jsx
cd px4
roslaunch px4 indoor1.launch

```

Open another terminal

```jsx
cd ~/catkin_ws
bash scripts/xtdrone_run_vio.sh

```

Open another terminal

```jsx
cd ~/XTDrone/sensing/slam/vio
python vins_transfer.py iris 0

```

Open another terminal

```jsx
cd ~/XTDrone/communication
python multirotor_communication.py iris 0

```

Open another terminal

```jsx
cd ~/XTDrone/control/keyboard
python multirotor_keyboard_control.py iris 1 vel

```