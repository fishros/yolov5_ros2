# YoloV5_ROS2

基于YoloV5的ROS2封装，给定模型和相机参数可以直接发布三维空间置进行抓取操作。

## 安装

```
sudo apt update
sudo apt install python3-pip
pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple yolov5
```
Ros2Pkgs

```
sudo apt install ros-humble-vision-msgs
```

## 编译运行

```
ros2 run yolov5_ros2 yolo_detect_2d -p device:=cpu
```