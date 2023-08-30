# ros2-topic-perf

- Alternative tool to measure performance metrics of ROS 2 topics: `Hz` and `Bandwidth`
- If you want to measure `Delay`, please refer to [`ros2 topic delay $TOPIC_NAME`](https://github.com/ros2/ros2cli/blob/humble/ros2topic/ros2topic/verb/delay.py) after installing `chrony`. (`sudo apt install chrony`, See [this article](https://wiki.nps.edu/display/RC/Time+Synchronisation+with+NTP) for more information.)

## 🧰 Using Docker

### Build

Build the docker image:

```bash
docker build -t ros2-topic-perf:latest ./docker
```

### Run

Run the docker container:

```bash
docker run --rm -it --network host --ipc host \
    -v /etc/timezone:/etc/timezone:ro \
    -v /etc/localtime:/etc/localtime:ro \
    ros2-topic-perf:latest \
    bash -c "ros2 run topic_perf monitor $TOPIC_NAME --ros-args -p time_window:=$TIME_WINDOW_SEC -p qos:=$QOS_PROFILE"
```

or simply run the script:

```bash
cd $PKG_PATH/docker
./run.sh $TOPIC_NAME $TIME_WINDOW_SEC $QOS_PROFILE
```

### Example Output

```bash
$ cd $PKG_PATH/docker
$ ./run.sh /rgb/image_raw 3 0
[INFO] [1693156941.645706671] [monitor]: time_window: 3
[INFO] [1693156941.645748836] [monitor]: qos_profile: 0 (default)
[INFO] [1693156941.645754400] [monitor]: topic_name: /rgb/image_raw
[INFO] [1693156941.645758031] [monitor]: Waiting 10 sec for topic...
[INFO] [1693156942.145897231] [monitor]: .
[INFO] [1693156942.146206065] [monitor]: topic_type: sensor_msgs/msg/Image
[WARN] [1693156943.147969671] [monitor]: (window: 3 [s]) hz: 1.7, bw: 20971620.0 [Byte/s] <- [Invalid] Data too small for the window.
[WARN] [1693156944.147968502] [monitor]: (window: 3 [s]) hz: 3.3, bw: 41943240.0 [Byte/s] <- [Invalid] Data too small for the window.
[WARN] [1693156945.147990045] [monitor]: (window: 3 [s]) hz: 5.0, bw: 62914860.0 [Byte/s] <- [Invalid] Data too small for the window.
[INFO] [1693156946.147976779] [monitor]: (window: 3 [s]) hz: 5.0, bw: 62914860.0 [Byte/s]
[INFO] [1693156947.147972053] [monitor]: (window: 3 [s]) hz: 5.0, bw: 62914860.0 [Byte/s]
[INFO] [1693156948.147931569] [monitor]: (window: 3 [s]) hz: 5.0, bw: 62914860.0 [Byte/s]
[INFO] [1693156949.148013788] [monitor]: (window: 3 [s]) hz: 5.0, bw: 62914860.0 [Byte/s]
[INFO] [1693156950.148106233] [monitor]: (window: 3 [s]) hz: 5.0, bw: 62914860.0 [Byte/s]
[INFO] [1693156951.148028207] [monitor]: (window: 3 [s]) hz: 5.0, bw: 62914860.0 [Byte/s]
[INFO] [1693156952.147999271] [monitor]: (window: 3 [s]) hz: 5.0, bw: 62914860.0 [Byte/s]
```

## 🧰 Using from Source

### Build

```bash
cd $WORKSPACE/src
git clone https://github.com/ryul1206/ros2-topic-perf.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

### Run

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
