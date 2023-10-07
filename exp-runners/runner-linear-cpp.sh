#!/bin/bash

_start=1
_end=100


#echo "label,timestamp,cpu,mem" > ../data/resource-linear-cpp1.csv

# Starting up the Turtlesim
ros2 run turtlesim turtlesim_node&
TURTLE_PID=$!

for i in $(seq ${_start} ${_end})
do
    ./progress-bar.sh ${_start} ${_end} ${i}

    python3 ../energy-mon.py -p turtlesim_linear_cpp -a movefb -i linear_cpp3 &> ./log/linear3.log &
    PROC_PID=$!
    
    #sleep 1
    #timestamp=$(date +%s)
    #mem1=`./mem.sh $LINEAR_PID`
    #cpu1=`./cpu.sh $LINEAR_PID`
    #talker="linear_cpp1,$timestamp,$cpu1,$mem1"
    #echo $talker >> ../data/resource-linear-cpp1.csv

    while kill -0 $PROC_PID 2> /dev/null; do
        sleep 1
    done 
    
    sleep 1
done
kill -9 $TURTLE_PID
pkill turtlesim_node
