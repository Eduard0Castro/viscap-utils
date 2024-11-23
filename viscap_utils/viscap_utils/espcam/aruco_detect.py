import cv2
import cv2.aruco as aruco
from mirela_sdk.image_processing.camera.calibration.calibration import Calibration

dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
param = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, param)
cam_matrix, coef_distort = Calibration.get_camera_matrix_distortion()
cap = cv2.VideoCapture("http://192.168.138.124:81/stream")

while cv2.waitKey(1) & 0xFF != ord("q"):

    success, frame = cap.read()
    if not success: break

    # frame = cv2.flip(frame, 1)

    bbox, ids, reject = detector.detectMarkers(frame)

    if ids is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(bbox, 0.2, cam_matrix, coef_distort)
        x, y, z = tvecs [0][0]        
        cv2.drawFrameAxes(frame, cam_matrix, coef_distort, rvecs[0], tvecs[0], 0.40)

    frame = aruco.drawDetectedMarkers(frame, bbox, ids, (0, 0, 255))
    cv2.imshow("Espcam", frame)

cap.release()
cv2.destroyAllWindows()