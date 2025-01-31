from rclpy.node import Node
from pathlib import Path
import cflib.crtp as crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.swarm import Swarm, CachedCfFactory


class ViscapCrazyflie:

    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
    MULTIRANGER = "multiranger"
    PATH = Path(__file__).resolve().parent

    def __init__(self, node: Node) -> None:

        """
        Initialize the crazyflie's class objects for Crazyflie, SyncCrazyflie, MotionCommander and 
        Multiranger

        :param node (rclpy.node.Node): the ROS2 node to handle the crazyflie actions
        """

        self.node = node
        self.tools = {ViscapCrazyflie.MULTIRANGER: lambda : Multiranger(self.sync_crazyflie)}
        self.__initialized_tools = list()

    def init_crazyflie(self, tools: list = list(), single_crazyflie: bool = True) -> MotionCommander | None:

        """
        Initialize the drivers, crazyflie's communication settings and and return motion 
        commander instance if set the single_crazyflie parameter. None for otherwise

        :param tools (list): list with the tools to initialize 
        :param single_crazyflie (bool): True for a single crazyflie application, False for Swarm

        """

        crtp.init_drivers()
        self.cleaned = False
        motion = None


        if single_crazyflie:
            self.crazyflie = Crazyflie(rw_cache = f"{ViscapCrazyflie.PATH}/.cache")
            self.sync_crazyflie = SyncCrazyflie(link_uri = self.URI, cf = self.crazyflie)

            self.motion = MotionCommander(self.sync_crazyflie, 0.5)

            self.__initialized_tools.append(self.sync_crazyflie)
            self.__initialized_tools.append(self.motion)

            self.sync_crazyflie.open_link()

            motion = self.motion


        for tool in tools:
            action = self.tools.get(tool, None)
            if action:
                tool_ = action()
                tool_.__enter__()
                self.__initialized_tools.append(tool_)

            else: raise ValueError("Invalid tool name")

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
        Monitore the 5 sensors in the multiranger deck and avoid colision. You must initialize 
        Multiranger with create_multiranger function
        :param distance_range (float): the distance_range in meters
        """
        self.distance_range = distance_range
        self.node.get_logger().info("Multiranger monitor timer initialized")
        self.multiranger_timer = self.node.create_timer(0.1, self.__multiranger_monitor)
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
            self.node.destroy_timer(self.multiranger_timer)
            self.node.get_logger().info("Drone landed. Application finished")
            return

        self.node.get_logger().info(f"Moving with vel x: {velocity_x} e vel y: {velocity_y}")
        self.motion.start_linear_motion(velocity_x, velocity_y, 0)

    def __is_close(self, range: float) -> bool:
        if range is None: return False
        else: return range < self.distance_range

    def cleanup(self) -> None:

        """
        Close all the links that were opened, finish the tools applications
        and land the drone
        """

        if not self.cleaned: 
            [tool.__exit__(None, None, None) for tool in self.__initialized_tools]
            self.cleaned = True
        else: print("Already closed")

    
        