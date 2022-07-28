from yolov5 import YOLOv5



def main():
    # set model params
    model_path = "./yolov5s.pt"
    device = "cuda:0" # or "cpu"

    # init yolov5 model
    yolov5 = YOLOv5(model_path, device)

    # load images
    image1 = 'zidane.jpg'
    image2 = 'bus.jpg'

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

main()