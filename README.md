# fiware-ros-turtlesim
This ros package acts as a bridge between [FIWARE](https://www.fiware.org) and [ROS](http://wiki.ros.org/) through MQTT.

[![TravisCI Status](https://travis-ci.org/tech-sketch/fiware-ros-turtlesim.svg?branch=master)](https://travis-ci.org/tech-sketch/fiware-ros-turtlesim)

## Description
### `command_sender`
This ROS node receives a command from [FIWARE orion context broker](https://catalogue-server.fiware.org/enablers/publishsubscribe-context-broker-orion-context-broker) through MQTT.

When receiving a command, this node publish a series of `geometry_msgs/Twist` messages to `/turtle1/cmd_vel` topic. By doing so, `turtlesim` moves according to the received command.

### `attribute_receiver`
This ROS node subscribes for `/turtle1/temperature` topic.

When receiving a `std_msgs/Float32` message from its topic, this node publishes the received `temperature` to FIWARE orion context broker through MQTT.

## Requirement

**ROS kinetic**

## Install libraries

```bash
$ cd ~/ros_ws/src
$ git clone https://github.com/tech-sketch/fiware-ros-turtlesim.git
$ cd fiware-ros-turtlesim
$ pip install -r requirements/common.txt
```

## How to Run
1. start X server.
1. open a terminal and start `roscore`.

    ```bash
    terminal1:$ cd ~/ros_ws
    terminal1:$ source devel/setup.bash
    terminal1:$ roscore
    ```
1. open another terminal and start `turtlesim`.

    ```bash
    terminal2:$ cd ~/ros_ws
    terminal2:$ source devel/setup.bash
    terminal2:$ rosrun turtlesim turtlesim_node
    ```
1. configure `config/params.yaml`

    * set `mqtt.host` and `mqtt.port`
    * If necessary, set `mqtt.cafile`, `mqtt.sername` and `mqtt.password`
1. open another terminal and start `fiware-ros-turtlesim`.

    ```bash
    terminal3:$ cd ~/ros_ws
    terminal3:$ source devel/setup.bash
    terminal3:$ roslaunch fiware-ros-turtlesim fiware-ros-turtlesim.launch
    ```

## License

[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2018 TIS Inc.
