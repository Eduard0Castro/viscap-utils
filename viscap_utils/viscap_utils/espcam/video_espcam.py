import cv2
from pathlib import Path

path = f"{Path(__file__).resolve().parent}"
cap = cv2.VideoCapture("http://192.168.138.124:81/stream")

fcode = cv2.VideoWriter.fourcc(*"mp4v")
video_file_name = "/videos/filmagem.mp4"
videoDimension = (480,320)
frame_rate = 20.0

recordedVideo = cv2.VideoWriter(video_file_name, fcode, frame_rate, videoDimension)

while cv2.waitKey(1) & 0xFF != ord("q"):

    success, frame = cap.read()

    if not success: break
    print(frame.shape)

    # recordedVideo.write(frame)

    cv2.imshow("ESPCAM", frame)

cap.release()
cv2.destroyAllWindows()
