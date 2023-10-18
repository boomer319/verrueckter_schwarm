import logging
import time

# cflib

# scanning for Crazyflies instances
import cflib.crtp

# connect/send/receive data from a Crazyflie
from cflib.crazyflie import Crazyflie

# wrapper around the “normal” Crazyflie class
# handles the asynchronous nature of the Crazyflie API
# and turns it into blocking function.
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# representation of one log configuration that enables logging from the Crazyflie
from cflib.crazyflie.log import LogConfig

# synchronous access to log data from the Crazyflie
from cflib.crazyflie.syncLogger import SyncLogger


# URI of Crazyflie to connect to (param: CFLIB_URI)
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def param_stab_est_callback(name, value):
    print('The crazyflie has parameter ' + name + ' set at number: ' + value)

def simple_param_async(scf, groupstr, namestr):
    cf = scf.cf
    full_name = groupstr+ "." +namestr
    cf.param.add_update_callback(group=groupstr, name=namestr, cb=param_stab_est_callback)
    time.sleep(1)
    cf.param.set_value(full_name, 2)
    time.sleep(1)
    cf.param.set_value(full_name, 1)
    time.sleep(1)

def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(5)
    logconf.stop()

def simple_log(scf, logconf):
    
    with SyncLogger(scf, lg_stab) as logger:

        for log_entry in logger:
            
            # time.sleep(0.5)

            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            print('[%d][%s]: %s' % (timestamp, logconf_name, data))

            # break

def simple_connect():

    print("Yeah, I'm connected! :D")
    time.sleep(3)
    print("Now I will disconnect :'(")

if __name__ == '__main__':

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    group = "stabilizer"
    name = "estimator"

    # syncCrazyflie will create a synchronous Crazyflie instance with the specified link_uri
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        
        # simple_connect()
        # simple_log(scf, lg_stab)
        # simple_log_async(scf, lg_stab)
        simple_param_async(scf, group, name)