import cflib.crtp as crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.uri_helper import uri_from_env
from time import sleep

uri = uri_from_env(default="radio://0/80/2M/E7E7E7E7E8")
crazyflie = Crazyflie(rw_cache='.cache')
crtp.init_drivers()


with SyncCrazyflie(uri, crazyflie) as sync:
    try:
        with MotionCommander(sync, default_height=0.4) as mc:
            sleep(2.0)
            
            mc.forward(0.4, 0.15)
            mc.turn_left(90)

            mc.forward(0.4, 0.15)
            mc.turn_left(90)

            mc.forward(0.4, 0.15)            
            mc.turn_left(90)

            mc.forward(0.4, 0.15)
            mc.turn_left()

    except Exception:
            mc.land()


