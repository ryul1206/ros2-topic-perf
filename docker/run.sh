# All arguments have to be passed in order to run the script
if [ $# -ne 3 ];
  then
    echo "No arguments supplied. Please provide topic name as argument."
    echo "[ Usage ] ./run.sh <topic_name> <time_window_sec> <qos_profile>"
    echo "[Example] ./run.sh /rosout 3 0"
    exit 1
fi
TOPIC_NAME=$1
TIME_WINDOW_SEC=$2
QOS_PROFILE=$3

# Start docker container
docker run --rm \
    -it \
    --name ros2-topic-perf \
    --network host \
    --privileged \
    --ipc host \
    -v /etc/timezone:/etc/timezone:ro \
    -v /etc/localtime:/etc/localtime:ro \
    ros2-topic-perf:latest \
    bash -c "ros2 run topic_perf monitor $TOPIC_NAME --ros-args -p time_window:=$TIME_WINDOW_SEC -p qos:=$QOS_PROFILE"
