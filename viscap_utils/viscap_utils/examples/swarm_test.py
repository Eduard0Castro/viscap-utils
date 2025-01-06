import rclpy
from rclpy.node import Node
from viscap_utils.crazyflie.viscap_crazyflie import ViscapCrazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from time import sleep

class SwarmTest(Node):

    def __init__(self) ->None:
        super().__init__("swarm_test_node")
        self.uris = {'radio://0/80/2M/E7E7E7E7E7',
                     'radio://0/80/2M/E7E7E7E7E8'}
        
        self.crazyflie = ViscapCrazyflie(self)
        self.crazyflie.init_crazyflie_swarm(self.uris, self.square)

    def square(self, sync: SyncCrazyflie) -> None:
        sync.cf.param.set_value('stabilizer.controller', 1)
        box_size = 1
        flight_time = 2

        commander = sync.cf.high_level_commander

        commander.takeoff(0.5, 2.0)
        sleep(3)

        commander.go_to(box_size, 0, 0, 0, flight_time, relative=True)
        sleep(flight_time)

        commander.go_to(0, box_size, 0, 0, flight_time, relative=True)
        sleep(flight_time)

        commander.go_to(-box_size, 0, 0, 0, flight_time, relative=True)
        sleep(flight_time)

        commander.go_to(0, -box_size, 0, 0, flight_time, relative=True)
        sleep(flight_time)

        commander.land(0.0, 2.0)
        sleep(2)

        commander.stop()

def main():
    rclpy.init()
    try:
        swarm = SwarmTest()
        rclpy.spin(swarm)
    except KeyboardInterrupt:...
    except Exception as ex: swarm.get_logger().error(ex)
    finally:
        swarm.crazyflie.cleanup()
        swarm.destroy_node()

if __name__ == "__main__":
    main()