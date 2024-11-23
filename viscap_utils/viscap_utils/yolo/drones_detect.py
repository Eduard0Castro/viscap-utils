from ultralytics import YOLO
import cv2
from pathlib import Path


path = Path(__file__).resolve().parent

model_path = f"{path}/models/best.pt"
model = YOLO(model_path, task="detect")
cap = cv2.VideoCapture(path/"videos/drones.mp4")

fcode = cv2.VideoWriter.fourcc(*"mp4v")
video_file_name = path/"videos/resultado.mp4"
videoDimension = (1280, 720)
frame_rate = 25.0

recordedVideo = cv2.VideoWriter(video_file_name, fcode, frame_rate, videoDimension)

while cv2.waitKey(1) & 0xFF != ord("q"):

    success, img = cap.read()
    if not success: break

    try:
        results = model(img)
        print()

    except Exception as ex:
        print(f"Exception: {ex}")

    else:

        for result in results:

            if not result.boxes:
                    print("No drones in the image")
                    
            else:
                boxes = result.boxes.xyxy.cpu().numpy() 
                confidences = result.boxes.conf.cpu().numpy()
                class_ids = result.boxes.cls.cpu().numpy().astype(int)

                for box, confidence, class_id in zip(boxes, confidences, class_ids):
                    if len(box) < 4:
                        print(f"Invalid box lenght: {len(box)}")
                        

                    x1, y1, x2, y2 = map(int, box[:4])

                    if x1 < 0 or y1 < 0 or x2 > img.shape[1] or y2 > img.shape[0]:
                        print(f"Invalid coordinates: {(x1, y1, x2, y2)}")

                    if confidence > 0.4:

                        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

                        label = f'Drone ({confidence:.2f})'
                        cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.5, (0, 255, 0), 2)
            recordedVideo.write(img)   

cap.release()
cv2.destroyAllWindows()
        