#!/bin/bash

_start=1
_end=100

# echo "label,timestamp,cpu,mem" > ../data/resource-linear-py1.csv

# Starting up the Turtlesim
ros2 run turtlesim turtlesim_node&> ./log/turtlesim.log &
TURTLE_PID=$!

for i in $(seq ${_start} ${_end})
do
    ./progress-bar.sh ${_start} ${_end} ${i}
 
    python3 ../energy-mon.py -p turtlesim_linear_py -a movefb -i linear_py3 &> ./log/linear-py3.log &
    MONITOR_PID=$!
    PROC_PID=`ps ax | grep turtlesim_linear_py/movefb | head -1 | awk '{ print $1 }'`
    
    # sleep 1
    # timestamp=$(date +%s)
    # mem1=`./mem.sh $PROC_PID`
    # cpu1=`./cpu.sh $PROC_PID`
    # sleep 1
    # resrouces="linear_py,$timestamp,$cpu1,$mem1"
    # echo $resources >> ../data/resource-linear-py1.csv

    while kill -0 $MONITOR_PID 2> /dev/null; do
        sleep 1
    done 
    
    sleep 1
done

kill -9 $TURTLE_PID
pkill turtlesim_node