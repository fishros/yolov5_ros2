[中文文档](./README.md) | [English Document](./README_EN.md)

# YOLO_ROS2

基于YOLOV5 的ROS2封装，允许用户使用给定的模型文件和相机参数进行三维空间物体检测和抓取操作。

![YOLO_ROS2](https://img-blog.csdnimg.cn/592a90f1441f4a3ab4b94891878fbc55.png)

## 1. 安装依赖

首先，确保您已经更新了系统并且安装了必要的依赖。以下是一些安装步骤，其中`$ROS_DISTRO` 是您的ROS2发行版（例如：`foxy`、`galactic`）：

```bash
sudo apt update
sudo apt install python3-pip ros-$ROS_DISTRO-vision-msgs
pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple yolov5  
```

## 2. 编译和运行

编译项目并设置环境变量：

```bash
colcon build
source install/setup.bash
```

现在，您可以运行Yolo_ROS2节点。默认情况下，它将使用CPU来进行检测，使用名为`/image`的图像话题。您可以根据需要更改这些参数：

```bash
ros2 run yolov5_ros2 yolo_detect_2d --ros-args -p device:=cpu -p image_topic:=/image
```

如果您要使用真实相机，请修改默认的图像话题（`image_topic:=/image`），然后在另一个终端中运行以下命令来将相机图像转化为ROS话题：

```bash
ros2 run image_tools cam2image --ros-args -p width:=640 -p height:=480 -p frequency:=30.0 -p device_id:=-1
```

您也可以使用其他相机，例如`usb_cam`。在这种情况下，安装相应的包并运行`usb_cam`节点：

```bash
sudo apt-get install ros-<ros2-distro>-usb-cam # 安装
ros2 run usb_cam usb_cam_node_exe
```

![Yolo_ROS2相机](https://img-blog.csdnimg.cn/c65bed0b67694ed69776151c203bb950.png)

## 3. 订阅结果

Yolo_ROS2将检测结果发布到`/yolo_result`话题中，包括原始像素坐标以及归一化后的相机坐标系下的x和y坐标。您可以使用以下命令查看检测结果：

```bash
ros2 topic echo /yolo_result
```

![Yolo_ROS2检测结果](https://img-blog.csdnimg.cn/ac963f4226bf497790c0ef2fd8d942a3.png)

## 4. 更进一步使用

### 4.1 参数设置

在运行Yolo_ROS2节点时，您可以使用 `-p name:=value` 的方式来修改参数值。

#### 4.1.1 图像话题

您可以通过指定以下参数来更改图像话题：

```bash
image_topic:=/image
```

#### 4.1.2 计算设备设置

如果您有CUDA支持的显卡，可以选择以下参数来配置计算设备：

```bash
device:=cpu
```

#### 4.1.3 是否实时显示结果

您可以使用以下参数来控制是否实时显示检测结果。设置为`True`将实时显示结果，设置为`False`则不会显示：

```bash
show_result:=False
```

请注意，实时显示中的`cv2.imshow`可能会卡住。如果只需要验证结果，可以将此参数设置为`False`。

#### 4.1.4 切换不同Yolov5模型

默认情况下，Yolo_ROS2使用`yolov5s`模型。您可以通过以下参数来更改模型：

```bash
model:=yolov5m
```

#### 4.1.5 是否发布结果图像

如果您希望Yolo_ROS2发布检测结果的图像，请使用以下参数：

```bash
pub_result_img:=True
```

这将允许您通过`/result_img`话题查看检测结果的图像。

#### 4.1.5 相机参数文件

功能包默认从 /camera/camera_info 话题获取相机参数，在获取成功前，相机参数文件路径可以通过参数进行设置，参数为：camera_info_file，通过该参数可以设置文件路径，注意需要使用绝对目录：

```bash
camera_info_file:=/home/fishros/chapt9/src/yolov5_ros2/config/camera_info.yaml
```
