# Install librealsense SDK
Reference -[Install_SDK](distribution_linux.md)
- Register the server's public key:
```
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```

- Make sure apt HTTPS support is installed:
`sudo apt-get install apt-transport-https`

- Add the server to the list of repositories:
```
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
```

- Install the libraries (see section below if upgrading packages):  
  `sudo apt-get install librealsense2-dkms`  
  `sudo apt-get install librealsense2-utils`  
  The above two lines will deploy librealsense2 udev rules, build and activate kernel modules, runtime library and executable demos and tools.  

- Optionally install the developer and debug packages:  
  `sudo apt-get install librealsense2-dev`  
  `sudo apt-get install librealsense2-dbg` 
- Start SDK
  `realsense-viewer`
# Install ROS package for testing realsense
```
cd ~/catkin_ws/src
git clone https://github.com/intel-ros/realsense.git
cd ..
catkin_make
rospack profile
source devel/setup.sh
```
-catkin_make may report an error 'lack of ddynamic_reconfigure'.Download ddynamic_reconfigure from github
```
cd ~/catkin_ws/src
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
cd ..
catkin_make
```
Then you can use ros package of realsense-camera
```
source ~/catkin_ws/src/devel/setup.bash
roslaunch realsense2_camera rs_rgbd.launch
```
# Use realsense camera launch vins-fusion
1.Modify file of realsense 
 Find **~/catkin_ws/src/realsense/realsense2_camera/launch/rs_camera.launch** and modify the main part is :
```
<arg name="enable_gyro"         default="true"/>
<arg name="enable_accel"        default="true"/>
<arg name="unite_imu_method"          default="linear_interpolation"/>
<arg name="enable_sync"               default="true"/>
```
2.Run vins and realsense camera
I haven't done the Calibration work for the realsense camera, it will cause a big shift of camera track
- open  realsense camera
```
source ~/catkin_ws/devel/setup.bash
roslaunch realsense2_camera rs_camera.launch
```
- open vins rviz
```
source ~/catkin_ws/devel/setup.bash
roslaunch vins vins_rviz.launch
```
- run config file
```
source ~/catkin_ws/devel/setup.bash
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/yourconfig_path/your_config_file.yaml 
#my code is rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/src/VINS-Fusion-master/config/realsense_d455/realsense_stereo_imu_config.yaml
```