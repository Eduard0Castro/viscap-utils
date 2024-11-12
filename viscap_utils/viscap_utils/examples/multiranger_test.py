import rclpy
from rclpy.node import Node
from time import sleep
from viscap_utils.crazyflie.viscap_crazyflie import ViscapCrazyflie

class MultirangerTest(Node):

    def __init__(self) -> None:
        
        super().__init__("test_multiranger_node")

        self.crazyflie = ViscapCrazyflie(self)
        self.crazyflie.init_crazyflie()

        self.get_logger().info("Test Multiranger Node has been initialized")

    def teste_gate(self) -> None:

        self.multiranger = self.crazyflie.create_multiranger()
        self.crazyflie.motion.up(1.3)
        self.crazyflie.motion.forward(1.0)
        sleep(0.5)
        self.crazyflie.motion.down(0.6)
        
        while self.multiranger.front <= 0.15:
            self.crazyflie.motion.start_linear_motion(0, 0.5, 0)

        self.crazyflie.motion.forward(0.5)
        self.crazyflie.motion.land()
        

    def navigate_test(self) -> None:

        self.crazyflie.create_multiranger()
        self.crazyflie.multiranger_navigate()

def main(args = None):
    
    rclpy.init(args=args)

    try:
        test = MultirangerTest()
        test.navigate_test()
        # test.teste_gate()
        rclpy.spin(test)

    except KeyboardInterrupt as ki:
        print(ki)

    except Exception:
        pass

    finally:
        test.crazyflie.cleanup()
        test.destroy_node()

if __name__ == "__main__":
    main()


