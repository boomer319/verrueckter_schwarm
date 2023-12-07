# cloning this git repo

```
git clone --recurse-subodules https://github.com/boomer319/verrueckter_schwarm.git
```

# crazyflie configuration

The crazyflies have a number on their belly. The number corresponds to the last digits in their radio address:

- radio://0/80/2M/0xE7E7E7E701 for cf01
- radio://1/100/2M/0xE7E7E7E702 for cf02
- radio://0/80/2M/0xE7E7E7E703 for cf03
- radio://1/100/2M/0xE7E7E7E704 for cf04
- radio://0/80/2M/0xE7E7E7E705 for cf05
- radio://1/100/2M/0xE7E7E7E706 for cf06
- radio://0/80/2M/0xE7E7E7E707 for cf07
- radio://1/100/2M/0xE7E7E7E708 for cf08

0 and 1 in ...//0/... are the ids of the radio that shall be used

80 and 100 are the channels

2M is radio bandwith

look at verrueckter_schwarm/ros2_ws/src/verrueckterschwarm2/crazyflie/config/**crazyflies.yaml**...this is the radio configuration found there

# frequently needed & useful commands

## general

```
source /opt/ros/humble/setup.bash # can be added at the bottom of your .bashrc
```
```
source install/setup.bash # execute in your ros2_ws! can be added at the bottom of your .bashrc
```
```
ifconfig # find out the network adapter configuration of your device
```
```
ping IP-Address # test wether data transfer to specified IP works as quickly as intended
```

## drone movement/control/interaction

```
cfclient # open the crazyflie client
```
```
ros2 launch crazyflie launch.py backend:=cflib # starting crazyswarm2's crazyflie server with cflib backend
```
```
ros2 launch crazyflie launch.py # starting crazyswarm2's crazyflie server with cpp backend (this one is used in the course of this project)
```
```
export PYTHONPATH=~/verrueckter_schwarm/crazyflie-firmware/build:$PYTHONPATH # necessary for starting the crazyflie simulation
ros2 launch crazyflie launch.py backend:=sim # starting simulated server
```
```
ros2 run crazyflie_examples hello_world # in separate terminal
```
```
ros2 run crazyflie_waypoint_mission goTo # in separate terminal
```
**The main waypoint mission script can be found in verrueckter_schwarm/ros2_ws/src/verrueckterschwarm2/crazyflie_waypoint_mission/crazyflie_waypoint_mission/goTo.py. It is intended to make the drones go to points in space while maintaining their respective positions in the swarm and always having the cameras face the direction of flight. This was done to give the crazyflies the ability to avoid obstacles in later parts of the project.**

## python env

It is very practical to make a python environment for all your different use cases.

This will allow you to not disrupt any of your other sub-projects by messing up your python dependencies

A python environment was made for:

- working with the crazyflie in general (e.g. starting cfclient)
- following the [ai deck classification example](https://www.bitcraze.io/documentation/repository/aideck-gap8-examples/master/ai-examples/classification-demo/)
- working with the GapSDK
- Tutorial: https://www.digitalocean.com/community/tutorials/how-to-install-python-3-and-set-up-a-programming-environment-on-an-ubuntu-20-04-server

```
source ~/environments/cf/bin/activate # enter python environment
```
```
deactivate # exit python env
```

You can add an alias in your .bashrc. This way you dont need to type the command every time. You just need to use cf in the command line and the corresponding command is executed:

```
alias cf='source ~/environments/cf/bin/activate'
```
## lines added to my .bashrc

```
# ROS
source /opt/ros/humble/setup.bash # source ros
alias cs2='cd ~/verrueckter_schwarm/ros2_ws && source install/local_setup.bash' # source ros workspace
alias chooser='ros2 run crazyflie chooser.py' # makes it easy to read the battery voltage, restart crazyflies, select and deselect them. the alternative to selecting them in chooser.py would be to manually go to crazyflies.yaml and type true and false for each crazyflie.

# MoCap
alias NatNet='~/verrueckter_schwarm/NatNetSDKCrossplatform/build/sampleClient' # launching the sampleClient to see whether your PC receives the MoCap data thats streamed by NatNet.

# Python
alias cf='source ~/environments/cf/bin/activate' # Environment for working with the crazyflie
alias cf_py2='source ~/environments/cf_py2/bin/activate' # Environment for working with the crazyflie but python2
alias cfai='source ~/environments/cfai/bin/activate' # Environment for following the classification example with the ai deck
alias gap='source ~/environments/gap/bin/activate' # Environment for working with the GAP8 chip

# GAP
alias GAP_SDK='gap && source ~/verrueckter_schwarm/gap_sdk/sourceme.sh' # sourcing the chip configuration for working with the GapSDK
alias JTAG='export GAPY_OPENOCD_CABLE=~/verrueckter_schwarm/gap_sdk/utils/gap8-openocd/tcl/interface/ftdi/olimex-arm-usb-tiny-h.cfg' # needed whenever you are working with the JTAG debugger
```

# getting started from scratch

## set up git

https://git-scm.com/download/linux

https://git-scm.com/book/en/v2/Getting-Started-First-Time-Git-Setup

## install cfclient

https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/#inst-comp

https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/

```
sudo apt install git python3-pip libxcb-xinerama0
pip3 install --upgrade pip

pip3 install cfclient
```

## configure your usb permissions for crazyradio access

https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/

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

## update the crazyradio PA (needed when using the cpp backend of crazyswarm2)

I already did this on the two radios used in this project. Shouldn't be necessary if you don't plan on using more radios.

https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/building_flashing/#building-the-firmware

## launch cfclient

```
cfclient
```

used for:

- [**changing the radio adresses on each drone in order to use a different amount of radios**](https://forum.bitcraze.io/viewtopic.php?t=5325)
- updating the software on each crazyflie idivdually
- connecting xbox or ps controller to your pc to control the drone (can be done through an android app too)
- monitoring the battery voltage


## set up crazyswarm2

### set up ros2 humble

https://docs.ros.org/en/humble/Installation.html

### "install" crazyswarm2

https://imrclab.github.io/crazyswarm2/installation.html

#### important: step 3 in this case will be as follows (notice crazyswarm2 is missing, as my fork was already cloned recursively with this git):
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    git clone --branch ros2 --recursive https://github.com/IMRCLab/motion_capture_tracking.git

### set up config of crazyswarm2

I configured the drones in crazyflies.yaml. Each drone was assigned a starting position, mocap marker configuration and radio address.

https://imrclab.github.io/crazyswarm2/usage.html

### use the crazyswarm2 sim

This can be used to test your scripts before running them with the drones in real life. Depending on the power you have available this might run slow.

- [download the crazyflie firmware](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#cloning)
- [go through these steps](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#build-python-bindings)
- [start the sim](https://imrclab.github.io/crazyswarm2/usage.html#simulation)

## set up MoCap System and Motive Software

### general setup

In order for the motion capture streaming service "NatNet" to send data to the PC you are running the crazyswarm server from, you have to have them both in the same newtork or directly linked through an ethernet switch. A wifi router can be used as a switch too. Simply plug everything into the yellow ethernet ports. Find the IP corresponding to the network both your PC and the MoCap PC use. Choose that IP of the MoCap PC in Motive. Also put it in to verrueckter_schwarm/ros2_ws/src/verrueckterschwarm2/crazyflie/config/**motion_capture.yaml**

### Motive Streaming Settings
- enable
- local interface: use the IP address of the PC running Motive
    - the third group of numbers of the IP has to be the same as for the receiving machine 192.168.XYZ.103 (this group should correspond for all devices on the given router)
- Multicast
- turn on only labeled and unlabeled marker streaming, turn off the streaming of other things, e.g. Rigid Bodies. Crazyswarm2 only needs the pointcloud of said markers.
- The rest can be left as seen on the screenshot NatNetSettings.png

# Bookmarks

### Python

[How To Install Python 3 and Set Up a Programming Environment on an Ubuntu 20.04 Server | DigitalOcean](https://www.digitalocean.com/community/tutorials/how-to-install-python-3-and-set-up-a-programming-environment-on-an-ubuntu-20-04-server)

[Solve Python error: subprocess-exited-with-error | bobbyhadz](https://bobbyhadz.com/blog/python-note-this-error-originates-from-subprocess)

[How to Add Python to PATH – Real Python](https://realpython.com/add-python-to-path/#how-to-add-python-to-path-on-linux-and-macos)

### Git

[Git - First-Time Git Setup](https://git-scm.com/book/en/v2/Getting-Started-First-Time-Git-Setup)

[Git - Make local HEAD the new master - Stack Overflow](https://stackoverflow.com/questions/26891904/git-make-local-head-the-new-master)

[git - How to push changes to github without pull - Stack Overflow](https://stackoverflow.com/questions/19303962/how-to-push-changes-to-github-without-pull)

### ROS2 Humble

[Ubuntu (Debian packages) — ROS 2 Documentation: Humble documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

[Tutorials — ROS 2 Documentation: Humble documentation](https://docs.ros.org/en/humble/Tutorials.html)

### Sim

[Gazebo - Docs: Binary Ubuntu Install](https://gazebosim.org/docs/fortress/install_ubuntu)

[Gazebo Spin Motors | Bitcraze](https://www.bitcraze.io/documentation/repository/crazyflie-simulation/main/user_guides/gazebo_spin_motors/)

[Configuration — Crazyswarm 0.3 documentation](https://crazyswarm.readthedocs.io/en/latest/configuration.html#configure-external-tracking-system)

[Tutorials — Crazyswarm 0.3 documentation](https://crazyswarm.readthedocs.io/en/latest/tutorials/tutorials.html#tutorial-hover)

### crazyswarm2

### NatNet

[NatNet SDK - EXTERNAL OptiTrack Documentation](https://docs.optitrack.com/developer-tools/natnet-sdk)

[whoenig/NatNetSDKCrossplatform: Crossplatform version of the OptiTrack NatNet SDK](https://github.com/whoenig/NatNetSDKCrossplatform)

[Crazyswarm2: A ROS 2 testbed for Aerial Robot Teams — Crazyswarm2 1.0a1 documentation](https://imrclab.github.io/crazyswarm2/index.html)

[IMRCLab/crazyswarm2: A Large Quadcopter Swarm](https://github.com/IMRCLab/crazyswarm2)

[IMRCLab/libmotioncapture: Interface Abstraction for Motion Capture System APIs such as VICON or OptiTrack](https://github.com/IMRCLab/libmotioncapture)

[Optitrack to Crazyflie Set up - Bitcraze Forums](https://forum.bitcraze.io/viewtopic.php?t=4173)

[libmotioncapture/examples/python.py at main · IMRCLab/libmotioncapture](https://github.com/IMRCLab/libmotioncapture/blob/main/examples/python.py)

[Optitrack · IMRCLab/crazyswarm2 · Discussion #286](https://github.com/IMRCLab/crazyswarm2/discussions/286)

[Settings: Streaming - EXTERNAL OptiTrack Documentation](https://docs.optitrack.com/motive-ui-panes/settings/settings-streaming)

[Crazyswarm2 development | Bitcraze](https://www.bitcraze.io/2022/10/crazyswarm2-development/)

[Frequently Asked Questions — Crazyswarm2 1.0a1 documentation](https://imrclab.github.io/crazyswarm2/faq.html#how-do-crazyswarm2-and-crazyswarm-differ)

[How does crazyswarm2 process data? What are the outputs to crazyflies? Documentation on the inner workings and data flow. · IMRCLab/crazyswarm2 · Discussion #338](https://github.com/IMRCLab/crazyswarm2/discussions/338)

[Building and Flashing | Bitcraze](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#build-python-bindings)

[Collision avoidance by jpreiss · Pull Request #628 · bitcraze/crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware/pull/628)

[whoenig/uav\_trajectories: Helper scripts and programs for trajectories](https://github.com/whoenig/uav_trajectories)

[crazyswarm/ros\_ws/src/crazyswarm/scripts/backgroundComputation.py at master · USC-ACTLab/crazyswarm](https://github.com/USC-ACTLab/crazyswarm/blob/master/ros_ws/src/crazyswarm/scripts/backgroundComputation.py)

### cf client setup

[Installation Instructions | Bitcraze](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/)

[USB permissions | Bitcraze](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/)

### step-by-step tutorials

[Step-by-Step: Connecting, logging and parameters | Bitcraze](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/sbs_connect_log_param/)

[Step-by-Step: Motion Commander | Bitcraze](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/sbs_motion_commander/)

### bitcraze discussions

[bitcraze · Discussions · GitHub](https://github.com/orgs/bitcraze/discussions)

[crazyflie 2.1 > custom firmware > xyz.bin generating out of custom .config · bitcraze · Discussion #936](https://github.com/orgs/bitcraze/discussions/936)

[AI deck cannot be flashed with wifi example (old bootloader, flashing freezes) · bitcraze · Discussion #942](https://github.com/orgs/bitcraze/discussions/942)

### AI deck

[Releases · bitcraze/aideck-gap8-examples](https://github.com/bitcraze/aideck-gap8-examples/releases)

[Getting started with the AI deck | Bitcraze](https://www.bitcraze.io/documentation/tutorials/getting-started-with-aideck/)

[WiFi Video Streamer | Bitcraze](https://www.bitcraze.io/documentation/repository/aideck-gap8-examples/master/simple-examples/wifi-streamer/)

[Flashing | Bitcraze](https://www.bitcraze.io/documentation/repository/aideck-gap8-examples/master/infrastructure/flashing/)

[Classification Demo | Bitcraze](https://www.bitcraze.io/documentation/repository/aideck-gap8-examples/master/ai-examples/classification-demo/)

[AI deck 1.1 | Bitcraze](https://www.bitcraze.io/products/ai-deck/)

[PULP-DroNet: open source and open hardware artificial intelligence for fully autonomous navigation on Crazyflie | Bitcraze](https://www.bitcraze.io/2019/05/pulp-dronet-open-source-and-open-hardware-artificial-intelligence-for-fully-autonomous-navigation-on-crazyflie/)

[AI-deck Workshop 1 - YouTube](https://www.youtube.com/watch?v=o9asYPHxEB4)

[Deep Learning](https://www.deeplearningbook.org/)

[STM-GAP8 CPX communication Example | Bitcraze](https://www.bitcraze.io/documentation/repository/aideck-gap8-examples/master/other-examples/stm_gap8_cpx/)

### NanoFlowNet

[Obstacle Avoidance AI-deck | Bitcraze](https://www.bitcraze.io/tag/ai-deck/)

[2209.06918.pdf](https://arxiv.org/pdf/2209.06918.pdf)

[tudelft/nanoflownet: Code and trained networks of the paper "NanoFlowNet: Real-time Dense Optical Flow on a Nano Quadcopter"](https://github.com/tudelft/nanoflownet)

[semantic segmentation CNN STDC-Seg](https://www.mdpi.com/1424-8220/23/14/6514)

[UAVid Semantic Segmentation Dataset](https://uavid.nl/)

[\[1504.06852\] FlowNet: Learning Optical Flow with Convolutional Networks](https://arxiv.org/abs/1504.06852)

[FlowNet: Learning Optical Flow with Convolutional Networks - YouTube](https://www.youtube.com/watch?v=k_wkDLJ8lJE)

### GAP8

[GreenWaves-Technologies/gap\_sdk: SDK for Greenwaves Technologies' GAP8 IoT Application Processor](https://github.com/GreenWaves-Technologies/gap_sdk)

[PowerPoint Presentation](https://cms.tinyml.org/wp-content/uploads/talks2020/tinyML_Talks_Manuele_Rusci_201027.pdf)

[Machine Learning on GAP8 for tiny devices - YouTube](https://www.youtube.com/watch?v=12s16Ex11EU)

[gap\_sdk/tools/nntool/README.md at master · GreenWaves-Technologies/gap\_sdk](https://github.com/GreenWaves-Technologies/gap_sdk/blob/master/tools/nntool/README.md)

[gap\_sdk/tools/nntool at master · GreenWaves-Technologies/gap\_sdk](https://github.com/GreenWaves-Technologies/gap_sdk/tree/master/tools/nntool)

[GreenWaves-Technologies/gap\_sdk at release-v4.8.0](https://github.com/GreenWaves-Technologies/gap_sdk/tree/release-v4.8.0#getting-started-with-the-gap-sdk)

### crazyradio PA

[Build and flash radio | Bitcraze](https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/building_flashing/)

[USB and Radio protocol of the Crazyradio dongle | Bitcraze](https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/functional-areas/usb_radio_protocol/)

### crazyflie-firmware

[crazyflie-firmware](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/)

[CPX - Crazyflie Packet eXchange | Bitcraze](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/cpx/)

[index \[Bitcraze Wiki\]](https://wiki.bitcraze.io/)

[Bitcraze](https://github.com/bitcraze)

[Bitcraze - Crazyflie 101 EPFL-LIS lecture - part 2 (2020) - YouTube](https://www.youtube.com/watch?v=BXjSgCwb_bM)

[Guest Lecture at EPFL Aerial Robotics Course 2020 | Bitcraze](https://www.bitcraze.io/about/events/epfl2020/)

[Helping drone swarms avoid obstacles without hitting each other - EPFL](https://actu.epfl.ch/news/helping-drone-swarms-avoid-obstacles-without-hitti/)

[Crazyflie](https://robots.ros.org/crazyflie/)

[crazyflie API](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/api/cflib/crazyflie/)

[Tutorials | Bitcraze](https://www.bitcraze.io/documentation/tutorials/)

[User guides | Bitcraze](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/)

[Crazyflie Summer Project with ROS2 and Mapping | Bitcraze](https://www.bitcraze.io/2022/07/crazyflie-summer-project-with-ros2-and-mapping/)

[Motion Capture positioning | Bitcraze](https://www.bitcraze.io/documentation/system/positioning/mocap-positioning/)

[Customize firmware with kbuild | Bitcraze](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/kbuild/)

[Building and Flashing | Bitcraze](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#flashing)

[Install Docker Engine on Ubuntu | Docker Docs](https://docs.docker.com/engine/install/ubuntu/)

[Mellinger and PID controllers - Bitcraze Forums](https://forum.bitcraze.io/viewtopic.php?t=4826)
