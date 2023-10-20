#!/bin/bash

_start=1
_end=100 # number of repetitions

_listeners=20

source /opt/ros/humble/setup.bash
source ../install/setup.bash 

pkill python3
pkill talker
pkill listener
COUNT=0

for i in $(seq ${_start} ${_end})
do
    ./progress-bar.sh ${_start} ${_end} ${i}
 
    python3 ../energy-mon.py -p simple_pub_sub_py -a talker -i pub_py_$_listeners &> ./log/talker-cpp.log &
    TALKER_PID=$!
    sleep 0.5

    for ii in $(seq 1 ${_listeners})
    do
        ros2 run simple_pub_sub_py listener &
        LISTENER_PID=$!
        sleep 0.1
    done
    
    sleep 1
    T_PID=`ps ax | grep simple_pub_sub_py/talker | head -1 | awk '{ print $1 }'`
    
    COUNTER=0
    CPU=0.0
    MEM=0
    A_CPU=0
    A_MEM=0
    while kill -0 $LISTENER_PID 2> /dev/null; do 
        CURRENT_CPU=`ps -C talker -o %cpu | tail -1 | grep [0-9]`
        CURRENT_MEM=`pmap $T_PID | tail -1 | awk '{ print $2 }' | sed 's/K//'`
        CPU=`python3 -c "print (float($CPU) + float($CURRENT_CPU))"`
        MEM=`python3 -c "print (float($MEM) + float($CURRENT_MEM))"`
        sleep 0.1
        let COUNTER++
    done 
    TIME=$(date +%s)
    A_CPU=`python3 -c "print (float($CPU) / float($COUNTER))"`
    A_CPU=`python3 -c "print ('%.2f' % round($A_CPU, 2))"`
    A_MEM=`python3 -c "print (float($MEM) / float($COUNTER))"`
    A_MEM=`python3 -c "print ('%.2f' % round($A_MEM, 2))"`
    echo "$COUNT,pub_py_$_listeners,$TIME,$A_CPU,$A_MEM" >> ../data/resource-pubsub.csv
    let COUNT++
done