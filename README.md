# ros2-topic-perf

Alternative tool to measure performance metrics of ROS 2 topics (hz, bandwidth)

## TODO

- Delay measurement. We need to resolve the time synchronization issue between the publisher and subscriber. Please refer to [this article](https://wiki.nps.edu/display/RC/Time+Synchronisation+with+NTP) for more information.
- Dockerize

## Using Docker

Build the docker image:

```bash

```

Run the docker container:

```bash

```

## Build from Source

```bash
cd $WORKSPACE/src
git clone https://github.com/ryul1206/ros2-topic-perf.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

After building the package, you can run the tool as follows:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # or rmw_fastrtps_cpp

ros2 run topic_perf monitor /topic_name --ros-args -p time_window:=5 -p qos:=2
```

- `time_window` must be a positive integer in seconds. (default: 1)
- Available `qos` values are as follows:
  - 0: `rmw_qos_profile_default` (default)
  - 1: `rmw_qos_profile_system_default`
  - 2: `rmw_qos_profile_sensor_data`
  - 3: `rmw_qos_profile_services_default`
  - 4: `rmw_qos_profile_parameters`
  - 5: `rmw_qos_profile_parameter_events`
  - See [rmw_qos_profile_t](https://github.com/ros2/rmw/blob/humble/rmw/include/rmw/qos_profiles.h) for more details.

## References

This tool is based on the following repositories:

- [gbiggs/bandwidth_measurererer](https://github.com/gbiggs/bandwidth_measurererer)
- [ros2cli: hz, bw, delay](https://github.com/ros2/ros2cli/tree/humble/ros2topic/ros2topic/verb)
- [ros2 demos: topic_monitor](https://github.com/ros2/demos/blob/7ecb54e14e1726d543f77c798bf41e19efc1b9b4/topic_monitor/topic_monitor/scripts/topic_monitor.py)
