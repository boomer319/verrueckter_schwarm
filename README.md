# cloning this git
git clone --recurse-subodules https://github.com/boomer319/verrueckter_schwarm.git

# crazyflie configuration
- the crazyflies have a number on their belly
- the number corresponds to the last digits in their radio address:
    - radio://0/80/2M/0xE7E7E7E701 for cf01
    - radio://1/100/2M/0xE7E7E7E702 for cf02
    - radio://0/80/2M/0xE7E7E7E703 for cf03
    - radio://1/100/2M/0xE7E7E7E704 for cf04
    - radio://0/80/2M/0xE7E7E7E705 for cf05
    - radio://1/100/2M/0xE7E7E7E706 for cf06
    - radio://0/80/2M/0xE7E7E7E707 for cf07
    - radio://1/100/2M/0xE7E7E7E708 for cf08
- 0 and 1 are the id of the radio that shall be used
- 80 and 100 are the channels
- 2M is radio bandwith
- look at crazyflies.yaml...this is the radio configuration found there

# frequently needed commands
## general
```
source /opt/ros/humble/setup.bash # can be added at the bottom of your .bashrc
```
```
source install/setup.bash # execute in your ros2_ws! can be added at the bottom of your .bashrc
```
```
ifconfig # find out the IP address of your device
```
```
ping IP-Address # test wether data transfer to specified IP works as quickly as intended
```
## drone movement/control/interaction
```
cfclient
```
```
ros2 launch crazyflie launch.py backend:=cflib # starting crazyswarm2's crazyflie server with cflib backend
```
```
ros2 launch crazyflie launch.py # starting crazyswarm2's crazyflie server with cpp backend
```
```
export PYTHONPATH=~/verrueckter_schwarm/crazyflie-firmware/build:$PYTHONPATH # necessary for starting the crazyflie simulation
ros2 launch crazyflie launch.py backend:=sim # starting simulated server
```
```
ros2 run crazyflie_examples hello_world # separate terminal
```
```
ros2 run crazyflie_waypoint_mission goTo # separate terminal
```
## python env
- https://www.digitalocean.com/community/tutorials/how-to-install-python-3-and-set-up-a-programming-environment-on-an-ubuntu-20-04-server
```
source ~/environments/cf/bin/activate # enter python environment
```
```
deactivate # exit python env
```

# getting started
## set up git
- https://git-scm.com/download/linux
- https://git-scm.com/book/en/v2/Getting-Started-First-Time-Git-Setup

## installing cfclient
https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/#inst-comp
https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/
```
sudo apt install git python3-pip libxcb-xinerama0
pip3 install --upgrade pip

pip3 install cfclient
```

## usb permissions for crazyradio access
- https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/
```
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER
```
log out / in
```
cat <<EOF | sudo tee /etc/udev/rules.d/99-bitcraze.rules > /dev/null
# Crazyradio (normal operation)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
# Crazyflie (over USB)
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
EOF
```
reloading udev-rules:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## updating crazyradio PA (needed when using the cpp backend of crazyswarm2)
- https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/building_flashing/#building-the-firmware

## launching cfclient
```
cfclient
```
- possibilities
    - connect xbox or ps controller to pc to control the drone
    - monitor the battery voltage
    - update each crazyflie idivdually
- changing the radio adresses in order to use one instead of two radios(https://forum.bitcraze.io/viewtopic.php?t=5325)

## set up crazyswarm2

### set up ros2 humble

- https://docs.ros.org/en/humble/Installation.html

### "install" crazyswarm2

- https://imrclab.github.io/crazyswarm2/installation.html

#### important: step 3 in this case will be (notice crazyswarm2 is missing, as it is already cloned recursively with this git):
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    git clone --branch ros2 --recursive https://github.com/IMRCLab/motion_capture_tracking.git

### set up MoCap System and Motive Software

#### general setup
- In order for the motion capture streaming service "NatNet" to send data to the PC you are running the crazyswarm server from, you have to have them both in the same newtork or directly linked through an ethernet switch.
- A wifi router can be used as a switch too. Simply plug everything into the yellow ethernet ports.

#### Motive Streaming Settings
- enable
- local interface: use the IP address of the PC running Motive
    - the third group of numbers of the IP has to be the same as for the receiving machine 192.168.XYZ.103 (this group should correspond for all devices on the given router)
- Multicast
- turn on only labeled and unlabeled marker streaming, turn off the streaming of other things, e.g. Rigid Bodies. Crazyswarm2 only needs the pointcloud of said markers.
- The rest can be left as seen on the screenshot NatNetSettings.png

### set up config of crazyswarm2

- https://imrclab.github.io/crazyswarm2/usage.html

### use the crazyswarm2 sim

- download the crazyflie firmware (https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#cloning)
- go through these steps (https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#build-python-bindings)
- start the sim (https://imrclab.github.io/crazyswarm2/usage.html#simulation)
