#!/bin/bash

# Launch file to start both minimal publisher and subscriber nodes
cleanup() {
    echo "Restarting ROS 2 daemon to clean up before shutting down all processes"
    ros2 daemon stop
    sleep 1
    ros2 daemon start
    echo "Shutting down all processes started by this script"
    kill 0
    exit
}

trap 'cleanup' SIGINT

#Launch the publisher nodes in the background
ros2 run ros2_fundamentals_examples py_minimal_publisher.py &
sleep 2
#Launch the subscriber nodes in the background
ros2 run ros2_fundamentals_examples py_minimal_subscriber.py