import cv2
import logging
from ultralytics import YOLO
from pathlib import Path
from glob import glob
from numpy.random import randint


class ObjectDetect:

    """

    Class for applying an YOLO object detection AI model.
    The class can perform detection by drawing the bounding boxes in images and
    videos. It requires the complete path for the methods applications. 
    You can use pathlib to get this.


    """

    def __init__(self, model_path: str, conf_threshold: float) -> None:

        """

        ObjectDetect constructor: class contructor to initialize the model, the
        confidence threshold and the color for each object class of the model.

        :param model_path (str): complete path of the trained model file which will be used 
         for object detection. 
        :param conf_threshold (float): confidence threshold for filtering detected 
         objects. Detections with a confidence score below this value will be discarded.
        
        """

        try: self.model = YOLO(model_path, task = "detect")
        except Exception as ex:
            logging.error(f"Failed to load the model: {ex}")
            raise RuntimeError("Error loading model. Verify the path and file integrity")
        
        self.conf_threshold = conf_threshold
        self.colors = randint(0, 255, size = (len(self.model.names), 3), dtype = "uint8")
        self.text_size = 0.5


    def detect_from_image(self, 
                          images_path_in: str, 
                          images_path_out: str) -> None:

        """

        Function to apply the trained model to the images from input folder 
        and save the results in the output folder.
        :param images_path_in (str): the complete folder path that the images are stored 
        :param images_path_out (str): the complete path that will be stored the result images

        """

        images = sorted(glob(f"{images_path_in}/*"))

        if not images: 
            logging.warning("No images from input directory path")
            return

        cont = 0

        for image in images:
            if not image.endswith(".txt"):
                img = cv2.imread(image)

                if img is None: 
                    logging.warning(f"Failed to load image: {image}")
                    continue

                output, _ = self.detection(img)
                cont += 1
                cv2.imwrite(f"{images_path_out}/output_{cont}.jpg", output)


    def detect_from_video(self, 
                          video_path: str,
                          output_video_path: str, 
                          frame_rate: float,
                          video_dimension: tuple[int, int]) -> None:

        """

        Function to apply the trained model to the input video and save
        the result. Both of videos must be in the videos folder

        :param video_path (str): the path of the video that will be sent to the model
        :param output_video_path (str): path of the output video with the detection
        :param frame_rate (float): frames per second for output video
        :param video_dimension (tuple[int, int]): tuple with width and height for output video

        """
        
        if not Path(video_path).exists():
            logging.error(f"Video was not found from path: {video_path}")
            return

        cap = cv2.VideoCapture(video_path)
        fcode = cv2.VideoWriter.fourcc(*"mp4v")
        recordedVideo = cv2.VideoWriter(output_video_path, 
                                        fcode, 
                                        frame_rate, 
                                        video_dimension)

        while True:

            success, img = cap.read()
            if not success: break

            detected, _ = self.detection(img)
            recordedVideo.write(detected)

        cap.release()
        recordedVideo.release()


    def detection(self, img: cv2.Mat) -> tuple[cv2.Mat, list[tuple]]:

        """

        Function to perform detection and draw the bouding boxes, model classes and
        the confidence in the image. Returns a tuple with the final image and a list 
        of image coordinates for each object detected

        :param img (cv2.Mat): the input cv2 image to perform the detection and 
         draw the results

        """

        detection_coordinates = list()
        try: results = self.model(img, conf = self.conf_threshold)
        except Exception as ex: logging.error(f"Error during detection: {ex}")

        else:

            for result in results:
                if len(result.boxes) == 0: continue

                boxes = result.boxes.xyxy.cpu().numpy() 
                confidences = result.boxes.conf.cpu().numpy()
                class_ids = result.boxes.cls.cpu().numpy().astype(int)

                for box, confidence, class_id in zip(boxes, confidences, class_ids):
                    object_label = self.model.names[class_id]
                    label = f"{object_label} ({confidence:.2f})"
                    x1, y1, x2, y2 = map(int, box[:4])
                    detection_coordinates.append((x1, y1, x2, y2))
                    color = [int(c) for c in self.colors[class_id]]

                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, self.text_size, 1)
                    cv2.rectangle(img, (x1 - 1, y1 - th - 5), (x1 + tw, y1), color, -1)
                    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 
                                self.text_size, (255, 255, 255), 1)
        
        finally: return img, detection_coordinates


if __name__ == "__main__":

    path = Path(__file__).resolve().parent
    object_detect = ObjectDetect(f"{path}/models/drones_model.pt", 0.3)

    object_detect.detect_from_image(images_path_in = f"{path}/media/images/input",
                                    images_path_out = f"{path}/media/images/output")
    object_detect.detect_from_video(video_path = f"{path}/media/videos/drones.mp4",
                                    output_video_path = f"{path}/media/videos/drones_resultado.mp4", 
                                    frame_rate = 25.0, 
                                    video_dimension = (1280, 720))
