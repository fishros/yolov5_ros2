from yolov5 import YOLOv5
import rclpy 
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from ament_index_python.packages import get_package_share_directory

# may raise PackageNotFoundError
package_share_directory = get_package_share_directory('yolov5_ros2')

class YoloV5Ros2(Node):
    def __init__()->None:
        pass
        # 1.load model
        # 2.create yolov5 obj and create publisher
        # 3.create sub image (if 3d, sub depth, if 2dpose,load camera info)
        # 4.convert cv2 (cvbridge)
        # 5.detect pub result 

def main():
    # set model params
    model_path = package_share_directory+"/config/yolov5s.pt"
    # device = "cuda:0" # or "cpu"
    device = "cpu" # or "cpu"
    # init yolov5 model
    yolov5 = YOLOv5(model_path, device)
    # load images
    image2 = package_share_directory+"/resource/mouse.png"
    image1 = package_share_directory+"/resource/fish.jpg"
    # image2 = 'bus.jpg'
    # perform inference
    results = yolov5.predict(image1)
    # perform inference with larger input size
    results = yolov5.predict(image1, size=1280)
    # perform inference with test time augmentation
    results = yolov5.predict(image1, augment=True)
    # perform inference on multiple images
    results = yolov5.predict([image1, image2], size=1280, augment=True)
    # parse results
    predictions = results.pred[0]
    boxes = predictions[:, :4] # x1, y1, x2, y2
    scores = predictions[:, 4]
    categories = predictions[:, 5]
    # show detection bounding boxes on image
    results.show()
    # save results into "results/" folder
    results.save(save_dir='results/')
