# Installation of PX4 and ROS on Ubuntu 18.04

First, note that during the download of Ubuntu 18.04, if you enter "y" and see an "abort" message, it's because Ubuntu 18.04 has issues recognizing multiline commands. The second line of the code is considered an input for the prompt. Simply enter the commands one by one to run them properly.

The installation process is mainly based on XTDrone's configuration documentation [Basic Configuration of Simulation Platform (yuque.com)](https://www.yuque.com/xtdrone/manual_cn/basic_config).

The report refers to [Basic Configuration Setup of Drone Simulation Platform Based on ROS and PX4 on Ubuntu 18.04_Drone Simulation Platform ros-CSDN Blog](https://blog.csdn.net/qq_45067735/article/details/107303796). This article is very detailed and can solve most problems.

Using ros-melodic.

# First, configure the Python dependencies

```jsx
sudo apt install ninja-build exiftool ninja-build protobuf-compiler libeigen3-dev genromfs xmlstarlet libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev python-pip python3-pip gawk

```

```jsx
pip2 install pandas jinja2 pyserial cerberus pyulog==0.7.0 numpy toml pyquaternion empy pyyaml
pip3 install packaging numpy empy toml pyyaml jinja2 pyargparse

```

If an error occurs, it's usually due to the pip version.

```jsx
pip install --upgrade setuptools

```

```jsx
python -m pip install --upgrade pip

```

Update pip version with these two lines of code, then install the Python dependencies again.

# Download ros-melodic

1. Change the download source

Tsinghua Source

```jsx
sudo sh -c '. /etc/lsb-release && echo "deb <http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/> lsb_release -cs main" > /etc/apt/sources.list.d/ros-latest.list'

```

USTC Source

```jsx
sudo sh -c '. /etc/lsb-release && echo "deb <http://mirrors.ustc.edu.cn/ros/ubuntu/> lsb_release -cs main" > /etc/apt/sources.list.d/ros-latest.list'

```

Official Source

```jsx
sudo sh -c 'echo "deb <http://packages.ros.org/ros/ubuntu> $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

```

1. Set the key

```jsx
sudo apt install curl
curl -s <https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc> | sudo apt-key add -

```

If an error occurs, run the following code

```jsx
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

```

1. Install

```jsx
sudo apt update
sudo apt install ros-melodic-desktop

```

Usually, you would choose to install ros-melodic-desktop-full, but this version has the wrong gazebo version. If related errors occur later, they are usually due to missing ROS tool packages. Follow system prompts to install them, and refer to chatGPT for solutions.

1. Set the environment

```jsx
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

```

This code runs the ROS startup command every time the terminal starts. The specific file can be found in the .bashrc file in the /home directory. This is a hidden file, so remember to check the option to show hidden files in the file manager settings.

1. Build package dependencies

```jsx
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep

```

```jsx
sudo rosdep init

```

1. Run rosdep update

```jsx
rosdep update

```

1. Start and check installation

```jsx
roscore

```

# Then install gazebo

1. Uninstall previous versions of gazebo. If you didn't download the full version, you can skip this step.

```jsx
sudo apt-get remove gazebo*
sudo apt-get remove libgazebo*
sudo apt-get remove ros-melodic-gazebo* # Modify for kinetic or noetic

```

1. Set up the computer to accept software from [package.osrfoundation.org](http://package.osrfoundation.org/)

```jsx
sudo sh -c 'echo "deb <http://packages.osrfoundation.org/gazebo/ubuntu-stable> lsb_release -cs main" > /etc/apt/sources.list.d/gazebo-stable.list'
cat /etc/apt/sources.list.d/gazebo-stable.list
#if "deb <http://packages.osrfoundation.org/gazebo/ubuntu-stable> xenial main" appears, there are no problems

```

1. Set the key

```jsx
wget <https://packages.osrfoundation.org/gazebo.key> -O - | sudo apt-key add -
sudo apt-get update

```

1. Install gazebo9.1

```jsx
sudo apt-get install gazebo9=9.1*

```

1. Gazebo itself is independent of ROS, so you also need to install ROS's Gazebo plugins

```jsx
sudo apt install ros-melodic-gazebo9-*
sudo apt install ros-melodic-gazebo-*

```

1. Run the test

First, open a terminal and enter

```jsx
roscore

```

Then open another terminal and enter

```jsx
rosrun gazebo_ros gazebo

```

If it runs normally, there are no problems.

# **MAVROS installation**

```jsx
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras 		# for ros-melodic
wget <https://gitee.com/robin_shaun/XTDrone/raw/master/sitl_config/mavros/install_geographiclib_datasets.sh>
sudo chmod a+x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

```

Please wait for the shell file to run. It will end after about three lines of prompts.

# **PX4 configuration**

1. It is recommended to use gitee for download

```jsx
git clone <https://gitee.com/robin_shaun/PX4_Firmware>
cd PX4_Firmware
git checkout -b xtdrone/dev v1.11.0-beta1

```

Or use GitHub for download

```jsx
git clone <https://github.com/PX4/PX4-Autopilot.git>
mv PX4-Autopilot PX4_Firmware
cd PX4_Firmware
git checkout -b xtdrone/dev v1.11.0-beta1
git submodule update --init --recursive
make px4_sitl_default gazebo

```

Updating submodules depends on the network and git cache settings. If you encounter a curl18 error, try increasing the cache size.

**sudo git config --global http.postBuffer 5242880000**

**sudo git config --global https.postBuffer 5242880000**

Set the cache to 5G.

1. Install necessary Python dependencies:

```jsx
cd ~/PX4_Firmware
bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

```

You may encounter issues with the sympy version. The solution may involve using conda to configure different environments. The reason is:

sympy requires a version greater than 1.10.1, which needs Python 3.7. However, Ubuntu 18.04 supports Python 3.6. Changing the source to Python 3.7 causes many dependencies to fail. I have tried modifying the sympy version to 1.9 in the configuration files, but it may affect the later make process (possibly not, as it succeeded once in the root environment). The errors during the make process can be numerous and complex, so it is recommended to install with chat assistance.

1. Compile

make px4_sitl_default gazebo

1. Open .bashrc and add the following code at the bottom (note the file paths):

```jsx
source ~/catkin_ws/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo

```

If the configuration here is incorrect, errors will occur when running roslaunch later.

Then open a terminal and source it:

```jsx
source ~/.bashrc

```

1. Run the following command to open gazebo:

```jsx
cd ~/PX4_Firmware
roslaunch px4 mavros_posix_sitl.launch

```

1. Open another terminal and run:

```jsx
rostopic echo /mavros/state

```

Check if mavros is running normally.

# Ground Station Installation

https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html

It is difficult to compile QGroundControl version 4 on Ubuntu 18.04, so it is recommended to use version 3.5.

https://github.com/mavlink/qgroundcontrol/releases/tag/v3.5.6

```jsx
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage

```