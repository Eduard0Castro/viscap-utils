from rclpy.node import Node
from pathlib import Path
import cflib.crtp as crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.swarm import Swarm, CachedCfFactory
from viscap_utils.crazyflie.ai_deck import AIDeck


class ViscapCrazyflie:

    PATH = Path(__file__).resolve().parent

    def __init__(self, node: Node) -> None:

        """
        Initialize the crazyflie's class objects for Crazyflie, SyncCrazyflie, MotionCommander and 
        Multiranger

        :param node (rclpy.node.Node): the ROS2 node to handle the crazyflie actions

        """

        self.node = node
        self.__initialized_tools = list()
        self.__timers = list()
        self.cleaned = False


    def init_crazyflie(self, 
                       single_crazyflie: bool = True,
                       uri: str = "radio://0/80/2M/E7E7E7E7E7") -> MotionCommander | None:

        """
        Initialize the drivers, crazyflie's communication settings and and return motion 
        commander instance if set the single_crazyflie parameter. None for otherwise

        :param single_crazyflie (bool): True for a single crazyflie application, False for Swarm
        :param uri (str): the uri to identify the crazyflie like "radio://0/80/2M/E7E7E7E7E7"

        """

        crtp.init_drivers()
        motion = None
        self.uri = uri_helper.uri_from_env(default = uri)


        if single_crazyflie:
            self.crazyflie = Crazyflie(rw_cache = f"{ViscapCrazyflie.PATH}/.cache")
            self.sync_crazyflie = SyncCrazyflie(link_uri = self.uri, cf = self.crazyflie)

            self.motion = MotionCommander(self.sync_crazyflie, 0.5)

            self.__initialized_tools.append(self.sync_crazyflie)
            self.__initialized_tools.append(self.motion)

            self.sync_crazyflie.open_link()

            motion = self.motion

        return motion


    def init_crazyflie_swarm(self, uris: set[str], func: callable) -> None:

        """
        Initialize crazyflie for a Swarm application
        
        :param uris (set[str]): A set of uris to use when connecting to the Crazyflies in the swarm
        :param func (callable): Function to execute for all Crazyflies in the swarm.
        """
        
        self.init_crazyflie(single_crazyflie = False)
        self.swarm = Swarm(uris = uris, factory = CachedCfFactory(rw_cache = ".cache"))
        self.__initialized_tools.append(self.swarm)

        self.swarm.open_links()
        self.swarm.reset_estimators()
        self.swarm.parallel_safe(func)

        
    def create_multiranger(self) -> Multiranger:

        """
        Create, start and return the multiranger object
        """

        self.multiranger = Multiranger(self.sync_crazyflie)
        self.multiranger.start()
        self.__initialized_tools.append(self.multiranger)

        return self.multiranger

    def multiranger_navigate(self,
                             distance_range: float) -> None:

        """
        Monitore the 5 sensors in the multiranger deck and avoid colision.
        :param distance_range (float): the distance_range in meters
        """

        self.distance_range = distance_range
        self.create_multiranger()
        self.node.get_logger().info("Multiranger monitor timer initialized")
        self.multiranger_timer = self.node.create_timer(0.1, self.__multiranger_monitor)
        self.__timers.append(self.multiranger_timer)
        self.__vel = 0.5

    def __multiranger_monitor(self) -> None:

        velocity_x = 0
        velocity_y = 0

        if self.__is_close(self.multiranger.front): velocity_x -= self.__vel
        if self.__is_close(self.multiranger.back) : velocity_x += self.__vel
        if self.__is_close(self.multiranger.right): velocity_y += self.__vel
        if self.__is_close(self.multiranger.left) : velocity_y -= self.__vel
        if self.__is_close(self.multiranger.up)   : 
            self.motion.land()
            self.__timers.remove(self.multiranger_timer)
            self.node.destroy_timer(self.multiranger_timer)
            self.node.get_logger().info("Drone landed. Application finished")
            return

        self.node.get_logger().info(f"Moving with vel x: {velocity_x} e vel y: {velocity_y}")
        self.motion.start_linear_motion(velocity_x, velocity_y, 0)

    def __is_close(self, range: float) -> bool:
        if range is None: return False
        else: return range < self.distance_range


    def create_ai_deck(self) -> AIDeck:

        """
        Create an AIDeck object to get the image and do the processing
        
        """

        self.ai_deck = AIDeck()
        self.ai_deck.connect_to_socket()  
        self.__initialized_tools.append(self.ai_deck)

    def processing_ai_deck_image(self, callback: callable) -> None:
        
        """
        Function to get the image from ai deck and call the callback to
        process the image
        
        :param callback (callable): the callback function to process the image
        
        """
        self.__ai_deck_callback = callback
        self.ai_deck_timer = self.node.create_timer(0.0001, self.__get_ai_deck_image)
        self.__timers.append(self.ai_deck_timer)


    def __get_ai_deck_image(self) -> None:
        img = self.ai_deck.get_image()

        if img is not None: self.__ai_deck_callback(img)
        else: raise ValueError("Error: image from ai deck get image is invalid")

   
    def cleanup(self) -> None:

        """
        Close all the links that were opened, finish the tools applications
        and land the drone.
        """

        if not self.cleaned: 
            [self.node.destroy_timer(timer) for timer in self.__timers]
            [tool.__exit__(None, None, None) for tool in self.__initialized_tools]
            self.__timers.clear()
            self.__initialized_tools.clear()
            self.cleaned = True

        else: print("Already closed")


    def __del__(self) -> None: 
        if not self.cleaned: self.cleanup()