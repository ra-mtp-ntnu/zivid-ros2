# Zivid ROS2 driver

This is the *in*official ROS2 driver for [Zivid 3D cameras](https://www.zivid.com/).


---

*Contents:*
[**Installation**](#installation) |
[**Getting Started**](#getting-started) |
[**Launching**](#launching-the-driver) |
[**Services**](#services) |
[**Topics**](#topics) |

---

<p align="center">
    <img src="https://www.zivid.com/software/zivid-ros/ros_zivid_camera.png" width="600" height="273">
</p>

## Installation

### ROS

This driver is currently known to be working on Ubuntu 18.04 with ROS2 Eloquent.

### Zivid SDK

To use the ROS driver you need to download and install the "Zivid Core" package.
Follow [this guide](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/59080712/Zivid+Software+Installation)
to install Zivid for your version of Ubuntu. "Zivid Studio" and "Zivid Tools" packages are
not required by the ROS driver, but can be useful for testing that your system has been setup correctly
and that the camera is detected.

An OpenCL 1.2 compatible GPU and OpenCL driver is required by the Zivid SDK. Follow
[this guide](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/426519/Install+OpenCL+drivers+on+Ubuntu) to
install OpenCL drivers for your system.

### C++ compiler

A C++17 compiler is required.

```bash
sudo apt-get install -y g++
```

### Downloading and building Zivid ROS driver

First, load the `setup.bash` script into the current session.

```bash
source /opt/ros/eloquent/setup.bash
```

Create the workspace and src directory:
```bash
mkdir -p ~/dev_ws/src
```

Clone the Zivid ROS project into the src directory:
```bash
cd ~/dev_ws/src
git clone https://github.com/zivid/zivid-ros.git
```

Install dependencies:
```bash
cd ~/dev_ws
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Finally, build the driver.

```bash
colcon build
```

## Getting started

Connect the Zivid camera to your USB3 port on your PC. You can use the ZividListCameras command-line
tool available in the "Zivid Tools" package to confirm that your system has been configured correctly, and
that the camera is discovered by your PC. You can also open Zivid Studio and connect to the camera.
Close Zivid Studio before continuing with the rest of this guide.


### Demo

The following command will start the camera node and RViz. Another node `capture_request` then configures and activate the camera. It then proceeds to call the `capture` service provided by the camera node at a 5 second interval. A point cloud is captured and published as a `PointCloud2` on the `/points` topic and should appear in RViz.
```
ros2 launch zivid_samples capture_rviz_demo.launch.py
```

You can also see the `PointCloud2` messages by running the following in a new terminal:
```
ros2 topic echo /points
```

### Demo manually


(*Note:* Rememeber to source the installation by running `source install/local_setup.bash`)  

Start from the root of your ROS workspace and run the camera node:
```
ros2 run zivid_camera manual_composition --ros-args --params-file src/zivid-ros/zivid_camera/config/zivid_camera.yml
```
Then run RViz and add the `PointCloud2` plugin and set the topic to `points`.  
Configure the camera:
```
ros2 lifecycle set /zivid_camera configure
```
Activate it:
```
ros2 lifecycle set /zivid_camera activate
```
Call the `capture` service:
```
ros2 service call /capture zivid_interfaces/srv/Capture
```


## Services

### capture_assistant/suggest_settings
[zivid_camera/CaptureAssistantSuggestSettings.srv](./zivid_camera/srv/CaptureAssistantSuggestSettings.srv)

Invoke this service to analyze your scene and find suggested settings for your particular scene,
camera distance, ambient lighting conditions, etc. The suggested settings are configured on this
node and accessible via dynamic_reconfigure, see section [Configuration](#configuration). When this
service has returned you can invoke the [capture](#capture) service to trigger a 3D capture using
these suggested settings.

This service has two parameters:

`max_capture_time` (duration):
> Specify the maximum capture time for the settings suggested by the Capture Assistant. A longer
> capture time may be required to get good data for more challenging scenes. Minimum value is
> 0.2 sec and maximum value is 10.0 sec.

`ambient_light_frequency` (uint8):
> Possible values are: `AMBIENT_LIGHT_FREQUENCY_NONE`, `AMBIENT_LIGHT_FREQUENCY_50HZ`,
> `AMBIENT_LIGHT_FREQUENCY_60HZ`. Can be used to ensure that the suggested settings are compatible
> with the frequency of the ambient light in the scene. If ambient light is unproblematic, use
> `AMBIENT_LIGHT_FREQUENCY_NONE` for optimal performance. Default is `AMBIENT_LIGHT_FREQUENCY_NONE`.


### capture
[zivid_camera/Capture.srv](./zivid_camera/srv/Capture.srv)

Invoke this service to trigger a 3D capture. See section [Configuration](#configuration) for how to
configure the 3D capture settings. The resulting point cloud is published on topic [points](#points),
color image is published on topic [color/image_color](#colorimage_color), and depth image is published
on topic [depth/image_raw](depthimage_raw). Camera calibration is published on topics
[color/camera_info](#colorcamera_info) and [depth/camera_info](#depthcamera_info).

### capture_2d
[zivid_camera/Capture2D.srv](./zivid_camera/srv/Capture2D.srv)

Invoke this service to trigger a 2D capture. See section [Configuration](#configuration) for how to
configure the 2D capture settings. The resulting 2D image is published to topic
[color/image_color](#colorimage_color). Note: 2D RGB image is also published as a part of 3D
capture, see [capture](#capture).


### camera_info/model_name
[zivid_camera/CameraInfoModelName.srv](./zivid_camera/srv/CameraInfoModelName.srv)

Returns the camera's model name.

### camera_info/serial_number
[zivid_camera/CameraInfoSerialNumber.srv](./zivid_camera/srv/CameraInfoSerialNumber.srv)

Returns the camera's serial number.

### is_connected
[zivid_camera/IsConnected.srv](./zivid_camera/srv/IsConnected.srv)

Returns if the camera is currently in `Connected` state (from the perspective of the ROS driver).
The connection status is updated by the driver every 10 seconds and before each [capture](#capture) service
call. If the camera is not in `Connected` state the driver will attempt to re-connect to the camera
when it detects that the camera is available. This can happen if the camera is power-cycled or the
USB cable is unplugged and then replugged.

## Topics

### color/camera_info
[sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)

Camera calibration and metadata.

### color/image_color
[sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)

Color/RGB image. For 3D captures ([capture](#capture) service) the image is encoded as "rgb8". For
2D captures ([capture_2d](#capture_2d) service) the image is encoded as "rgba8", where the alpha
channel is always 255.

### depth/camera_info
[sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)

Camera calibration and metadata.

### depth/image_raw
[sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)

Depth image. Each pixel contains the z-value (along the camera Z axis) in meters.
The image is encoded as 32-bit float. Pixels where z-value is missing are NaN.

### points
[sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)

Point cloud data. Each time [capture](#capture) is invoked the resulting point cloud is published
on this topic. The included point fields are x, y, z (in meters), c (contrast value),
and r, g, b (colors). The output is in the camera's optical frame, where x is right, y is
down and z is forward.


## License

This project is licensed under BSD 3-clause license, see the [LICENSE](LICENSE) file for details.

