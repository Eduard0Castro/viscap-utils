from rclpy.node import Node
import cflib.crtp as crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander


class ViscapCrazyflie:

    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
    MULTIRANGER = "multiranger"

    def __init__(self, node: Node) -> None:

        """
        Initialize the crazyflie's class objects for Crazyflie, SyncCrazyflie, MotionCommander and 
        Multiranger
        :param node (rclpy.node.Node): the ROS2 node to handle the crazyflie actions
        """

        self.node = node

        self.crazyflie = Crazyflie(rw_cache="./cache")
        self.sync_crazyflie = SyncCrazyflie(link_uri = ViscapCrazyflie.URI, cf = self.crazyflie)

        self.motion = MotionCommander(self.sync_crazyflie, 0.5)
        # self.position_hl = PositionHlCommander(self.sync_crazyflie)

        self.cleaned = False

        self.tools = {ViscapCrazyflie.MULTIRANGER: lambda : Multiranger(self.sync_crazyflie)}
        self.__initialized_tools = list()

    def init_crazyflie(self, tools: list, height: float = 0.5) -> None:

        """
        Initialize the drivers, crazyflie's communication settings and make it take off

        :param tools (list): list with the tools to initialize 
        :param height (float): the height in meters for drone takeoff 
        """

        crtp.init_drivers()
        self.sync_crazyflie.open_link()
        self.motion.take_off(height=height)

        for tool in tools:
            action = self.tools.get(tool, None)
            if action:
                tool_ = action()
                tool_.__enter__()
                self.__initialized_tools.append(tool_)

            else: raise ValueError("Invalid tool name")


    def create_multiranger(self) -> Multiranger:

        """
        Create, start and return the multiranger object
        """

        self.multiranger = Multiranger(self.sync_crazyflie)
        self.multiranger.start()
        self.__initialized_tools.append(self.multiranger)

        return self.multiranger

    def multiranger_navigate(self) -> None:

        """
        Monitore the 5 sensors in the multiranger deck and avoid colision. You must initialize 
        Multiranger with create_multiranger function
        """

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

        self.motion.start_linear_motion(velocity_x, velocity_y, 0)

    def __is_close(self, range: float) -> bool:
        if range is None: return False
        else: return range < 0.2

    def cleanup(self) -> None:

        if not self.cleaned: 
            self.motion.land()
            [tool.__exit__(None, None, None) for tool in self.__initialized_tools]
            self.sync_crazyflie.close_link()
            self.cleaned = True
        else: print("Already closed")

    
        