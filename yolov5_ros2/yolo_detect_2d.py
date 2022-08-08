from math import frexp
from traceback import print_tb
from torch import imag
from yolov5 import YOLOv5
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor

from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose, Detection2D
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

package_share_directory = get_package_share_directory('yolov5_ros2')
# package_share_directory = "/home/mouse/code/github/yolov5_test/src/yolov5_ros2"


class YoloV5Ros2(Node):
    def __init__(self):
        super().__init__('yolov5_ros2')
        self.declare_parameter("device", "cuda", ParameterDescriptor(
            name="device", description="calculate_device default:cpu optional:cuda:0"))

        self.declare_parameter("model", f"{package_share_directory}/config/yolov5s.pt", ParameterDescriptor(
            name="model", description=f"default: {package_share_directory}/config/yolov5s.pt"))

        self.declare_parameter("image_topic", "/camera/image_raw", ParameterDescriptor(
            name="image_topic", description=f"default: /camera/image_raw"))

        self.declare_parameter("camera_info_topic", "/camera/camera_info", ParameterDescriptor(
            name="camera_info_topic", description=f"default: /camera/camera_info"))

        # 默认从camera_info中读取参数,如果可以从话题接收到参数则覆盖文件中的参数
        self.declare_parameter("camera_info", "", ParameterDescriptor(
            name="camera_info", description=f"see:config/camera_info.yaml"))

        # 1.load model
        model = self.get_parameter('model').value
        device = self.get_parameter('device').value
        self.yolov5 = YOLOv5(model_path=model, device=device)

        # 2.create publisher
        self.yolo_result_pub = self.create_publisher(
            Detection2DArray, "yolo_result", 10)
        self.result_msg = Detection2DArray()

        # 3.create sub image (if 3d, sub depth, if 2d load camera info)
        image_topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 1)

        self.camera_info = {}

        # 4.convert cv2 (cvbridge)
        self.bridge = CvBridge()

    def camera_info_callback(self, msg: CameraInfo):
        """
        通过回调函数获取到相机的参数信息
        """
        self.camera_info['k'] = msg.k
        self.camera_info['p'] = msg.p
        self.camera_info['d'] = msg.d
        self.camera_info['r'] = msg.r
        self.camera_info['roi'] = msg.roi

        self.camera_info_sub.destroy()

    def image_callback(self, msg: Image):
        self.get_logger().info("get a image~")

        # 5.detect pub result
        image = self.bridge.imgmsg_to_cv2(msg)
        detect_result = self.yolov5.predict(image)

        self.result_msg.detections.clear()
        self.result_msg.header.frame_id = "camera"
        self.result_msg.header.stamp = self.get_clock().now().to_msg()

        print(detect_result, len(detect_result.pred))
        # detect_result.show()
        detect_result.save(save_dir='results/')
        # parse results
        predictions = detect_result.pred[0]
        boxes = predictions[:, :4]  # x1, y1, x2, y2
        scores = predictions[:, 4]
        categories = predictions[:, 5]

        for index in range(len(categories)):
            # print(detect_result.names[int(id)])
            name = detect_result.names[int(categories[index])]
            detection2d = Detection2D()
            detection2d.id = name
            # detection2d.bbox
            x1, y1, x2, y2 = boxes[index]
            detection2d.bbox.center.position.x = float(x1+x2)/2.0
            detection2d.bbox.center.position.y = float(y1+y2)/2.0
            detection2d.bbox.size_x = float(x2-x1)
            detection2d.bbox.size_y = float(y2-y1)

            obj_pose = ObjectHypothesisWithPose()
            obj_pose.hypothesis.class_id = name
            obj_pose.hypothesis.score = float(scores[index])
            # obj_pose.pose.pose.position.x = 0.0
            # obj_pose.pose.pose.position.y = 0.0
            # obj_pose.pose.pose.position.z = 1.0  #2D相机则显示,归一化后的结果,用户用时自行乘上深度z获取正确xy
            detection2d.results.append(obj_pose)
            self.result_msg.detections.append(detection2d)

        # if view or pub
        detect_result.show()
        # draw image
        # if view show
        # if pub publish

        if len(categories) > 0:
            self.yolo_result_pub.publish(self.result_msg)


def main():
    rclpy.init()
    rclpy.spin(YoloV5Ros2())
    rclpy.shutdown()
    # set model params
    # device = "cuda:0" # or "cpu"
    # device = "cpu" # or "cpu"
    # # init yolov5 model
    # yolov5 = YOLOv5(model_path, device)
    # # load images
    # image2 = package_share_directory+"/resource/mouse.png"
    # image1 = package_share_directory+"/resource/fish.jpg"
    # # image2 = 'bus.jpg'
    # # perform inference
    # results = yolov5.predict(image1)
    # # perform inference with larger input size
    # results = yolov5.predict(image1, size=1280)
    # # perform inference with test time augmentation
    # results = yolov5.predict(image1, augment=True)
    # # perform inference on multiple images
    # results = yolov5.predict([image1, image2], size=1280, augment=True)
    # # parse results
    # predictions = results.pred[0]
    # boxes = predictions[:, :4] # x1, y1, x2, y2
    # scores = predictions[:, 4]
    # categories = predictions[:, 5]
    # # show detection bounding boxes on image
    # results.show()
    # # save results into "results/" folder
    # results.save(save_dir='results/')


if __name__ == "__main__":
    main()
