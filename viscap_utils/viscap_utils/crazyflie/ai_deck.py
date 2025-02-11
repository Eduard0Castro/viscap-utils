import argparse
import socket, struct
import numpy as np
import cv2


class AIDeck():

    """
    Class to use the AI-deck with a Wi-Fi network and connection with sockets. 
    It works independently of cflib and has methods to get and proccess the image from
    the camera. It's based on the https://github.com/bitcraze/aideck-gap8-examples/ 
    repository by Bitcraze. 
    
    """
    
    def __init__(self) -> None:

        """
        AIDeck constructor:
        Set the args for IP/port of AI-deck

        """

        # Args for setting IP/port of AI-deck. Default settings are for when
        # AI-deck is in AP mode.
        parser = argparse.ArgumentParser(description='Connect to AI-deck streamer')
        parser.add_argument("-n",  default="192.168.4.1", metavar="ip", 
                            help="AI-deck IP")
        parser.add_argument("-p", type=int, default='5000', metavar="port", 
                            help="AI-deck port")
        parser.add_argument('--save', action='store_true', help="Save streamed images")
        
        self.args = parser.parse_args()

        
    def connect_to_socket(self) -> None:

        """
        Function to initialize the connection with socket setting the deck ip and port
        """
        
        deck_port = self.args.p
        deck_ip = self.args.n
        print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((deck_ip, deck_port))
        print("Socket connected")

    def __rx_bytes(self, size: int) -> bytes:
        data = bytearray()
        while len(data) < size:
            data.extend(self.client_socket.recv(size-len(data)))
        return data
    
    def get_image(self) -> cv2.typing.MatLike | None:

        """
        Function to get the images from AI-deck camera. It returns a MatLike
        
        """

        imgStream, format_ = self.__start_rx()
        if format_ == 0:
            bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
            bayer_img.shape = (244, 324)
            color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGR)
            return color_img
    
    def __start_rx(self):

        packetInfoRaw = self.__rx_bytes(4)

        [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
        imgHeader = self.__rx_bytes(length - 2)
        [magic, w, h, depth, format_, size] = struct.unpack('<BHHBBI', imgHeader)

        if magic == 0xBC: imgStream = bytearray()

        # Now we start rx the image, this will be split up in packages of some size
        while len(imgStream) < size:
            packetInfoRaw = self.__rx_bytes(4)
            [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
            chunk = self.__rx_bytes(length - 2)
            imgStream.extend(chunk)
        
        return imgStream, format_
    
    def __enter__(self) -> None: self.connect_to_socket()
    def __exit__(self, exc_type: Exception, exc_value, traceback):
        self.client_socket.close()


if __name__ == "__main__":

    ai_deck = AIDeck()
    ai_deck.connect_to_socket()

    try: 
        while cv2.waitKey(1) & 0XFF != ord("q"):
            cv2.imshow("Color", ai_deck.get_image())
        
    except KeyboardInterrupt:...
    except Exception as ex: print(f"Error: {ex}")
    finally: 
        ai_deck.client_socket.close()
        cv2.destroyAllWindows()
        print("Close application")
