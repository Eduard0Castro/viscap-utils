import cflib.crtp as crtp
from time import sleep
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.swarm import Swarm, CachedCfFactory


uris = {
    'radio://0/80/2M/E7E7E7E7E7',
    'radio://0/80/2M/E7E7E7E7E8',
    # Add more URIs if you want more copters in the swarm
}

factory = CachedCfFactory(rw_cache=".cache")
crtp.init_drivers()

def activate_mellinger_controller(scf: SyncCrazyflie, use_mellinger: bool) -> None:
    
    controller = 2 if use_mellinger else 1
    scf.cf.param.set_value('stabilizer.controller', controller)


def run_shared_sequence(scf: SyncCrazyflie):

    activate_mellinger_controller(scf, False)

    box_size = 1.5
    flight_time = 2

    commander = scf.cf.high_level_commander

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

with Swarm(uris=uris, factory=factory) as swarm:

        swarm.reset_estimators()
        swarm.parallel_safe(run_shared_sequence)




