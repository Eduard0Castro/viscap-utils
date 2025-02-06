import rclpy
from rclpy.node import Node
from viscap_utils.crazyflie.viscap_crazyflie import ViscapCrazyflie
import cv2


class AIDeckExample(Node):

    def __init__(self) -> None:

        super().__init__("ai_deck_example_node")
        self.crazyflie = ViscapCrazyflie(self)
        self.ai_deck = self.crazyflie.create_ai_deck()
        self.crazyflie.processing_ai_deck_image(self.stream_cam)
        
        fcode = cv2.VideoWriter.fourcc(*"mp4v")
        video_file_name = "filmagem.mp4"
        videoDimension = (324,244)
        frame_rate = 20.0
        self.recordVideo = cv2.VideoWriter(video_file_name, fcode, frame_rate, videoDimension)



    def stream_cam(self, img: cv2.typing.MatLike) -> None:
        
        frame = img
        cv2.imshow("Teste", frame)

        print(img.shape)

        self.recordVideo.write(frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.recordVideo.release()
            cv2.destroyWindow("Teste")
            self.crazyflie.cleanup()


def main(args = None) -> None:

    rclpy.init(args = args)
    try:
        ai_deck_example = AIDeckExample()
        rclpy.spin(ai_deck_example)

    except KeyboardInterrupt:...
    except Exception as ex: 
        print(ex)
        ai_deck_example.crazyflie.cleanup()
    finally: ai_deck_example.destroy_node()

if __name__ == "__main__":
    main()

