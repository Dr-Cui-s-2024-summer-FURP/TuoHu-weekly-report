# Resolving Frame Drop Issues with the D455 Camera in Ubuntu 18.04

The default installation of the librealsense SDK and the related ROS environment packages can cause the camera to drop frames.

https://github.com/IntelRealSense/librealsense/issues/13150

After checking the locally downloaded realsense-ROS package, it was found that SDK 2.50.0 needs to be used to meet the 5.13.0.50 realsense-ROS dependency, otherwise, frame drop issues will occur.

First, uninstall the existing realsense-SDK on your machine

```jsx
dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge

```

Then, download the new SDK

```
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

```

```jsx
sudo apt install librealsense2=2.50.0-0~realsense0.6128

```

If the corresponding version is not available, you can search for it using apt

```jsx
sudo apt-cache madison librealsense

```

Ensure that dkms and utils are also using the corresponding versions.