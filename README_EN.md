[中文文档](./README.md) |[English Document](./README_EN.md)

# YOLO_ROS2

Based on YOLOV5, YOLO_ROS2 is a ROS2 wrapper that allows users to perform 3D spatial object detection and grasping operations using provided model files and camera parameters.

![YOLO_ROS2](https://img-blog.csdnimg.cn/592a90f1441f4a3ab4b94891878fbc55.png)

## 1. Installation Dependencies

First, make sure you have updated your system and installed the necessary dependencies. Here are some installation steps, where `$ROS_DISTRO` is your ROS2 distribution (e.g., `foxy`, `galactic`):

```bash
sudo apt update
sudo apt install python3-pip ros-$ROS_DISTRO-vision-msgs
pip3 install yolov5  
```

## 2. Compilation and Execution

Compile the project and set the environment variables:

```bash
colcon build
source install/setup.bash
```

Now, you can run the Yolo_ROS2 node. By default, it will use the CPU for detection and the image topic named `/image`. You can change these parameters as needed:

```bash
ros2 run yolov5_ros2 yolo_detect_2d --ros-args -p device:=cpu -p image_topic:=/image
```

If you want to use a real camera, modify the default image topic (`image_topic:=/image`), and then run the following command in another terminal to convert the camera image into a ROS topic:

```bash
ros2 run image_tools cam2image --ros-args -p width:=640 -p height:=480 -p frequency:=30.0 -p device_id:=-1
```

You can also use other cameras, such as `usb_cam`. In that case, install the corresponding package and run the `usb_cam` node:

```bash
sudo apt-get install ros-<ros2-distro>-usb-cam # Installation
ros2 run usb_cam usb_cam_node_exe
```

![Yolo_ROS2 Camera](https://img-blog.csdnimg.cn/c65bed0b67694ed69776151c203bb950.png)

## 3. Subscribe to Results

Yolo_ROS2 publishes detection results to the `/yolo_result` topic, including raw pixel coordinates and normalized camera coordinates (x and y). You can view detection results using the following command:

```bash
ros2 topic echo /yolo_result
```

![Yolo_ROS2 Detection Results](https://img-blog.csdnimg.cn/ac963f4226bf497790c0ef2fd8d942a3.png)

## 4. Further Customization

### 4.1. Parameter Settings

When running the Yolo_ROS2 node, you can modify parameter values using the `-p name:=value` format.

#### 4.1.1 Image Topic

You can change the image topic by specifying the following parameter:

```bash
image_topic:=/image
```

#### 4.1.2 Compute Device Settings

If you have a CUDA-supported GPU, you can configure the compute device with the following parameter:

```bash
device:=cpu
```

#### 4.1.3 Real-Time Result Display

You can control whether to display detection results in real-time using the following parameter. Set it to `True` to display results in real-time, and `False` to disable it:

```bash
show_result:=False
```

Please note that real-time display with `cv2.imshow` may freeze. If you only need to validate results, you can set this parameter to `False`.

#### 4.1.4 Switching Different Yolov5 Models

By default, Yolo_ROS2 uses the `yolov5s` model. You can change the model using the following parameter:

```bash
model:=yolov5m
```

#### 4.1.5 Publishing Result Images

If you want Yolo_ROS2 to publish images of detection results, use the following parameter:

```bash
pub_result_img:=True
```

This will allow you to view images of detection results via the `/result_img` topic.

#### 4.1.5 Camera Parameter File

The package defaults to obtaining camera parameters from the `/camera/camera_info` topic. If the parameters are not available, you can set the camera parameter file path using the `camera_info_file` parameter. Make sure to use an absolute directory path:

```bash
camera_info_file:=/home/fishros/chapt9/src/yolov5_ros2/config/camera_info.yaml
```
