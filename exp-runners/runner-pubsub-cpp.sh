#!/bin/bash

_start=1
_end=100

#echo "exec,label,timestamp,cpu,mem" > ../data/resource-pubsub.csv

source /opt/ros/humble/setup.bash
source ../install/setup.bash 

pkill python3
COUNT=0

for i in $(seq ${_start} ${_end})
do
    ./progress-bar.sh ${_start} ${_end} ${i}
 
    python3 ../energy-mon.py -p simple_pub_sub_cpp -a talker -i pub_cpp4 &> ./log/talker-cpp.log &
    TALKER_PID=$!

    python3 ../energy-mon.py -p simple_pub_sub_cpp -a listener -i sub_cpp4 &> ./log/listener-cpp.log & 
    LISTENER_PID=$!
    
    sleep 1
    T_PID=`ps ax | grep ./runner-pubsub-cpp.sh | head -1 | awk '{ print $1 }'`
    L_PID=$T_PID
    
    COUNTER=0
    CPU=0.0
    MEM=0
    CPU_L=0.0
    MEM_L=0
    A_CPU=0
    A_MEM=0
    A_CPU_=0
    A_MEM_L=0
    while kill -0 $LISTENER_PID 2> /dev/null; do
        CURRENT_CPU=`ps -C talker -o %cpu | tail -1 | grep [0-9]`
        CURRENT_CPU_L=`ps -C listener -o %cpu | tail -1 | grep [0-9]`
        #echo $CURRENT_CPU
        CURRENT_MEM=`pmap $T_PID | head -3 | tail -1 | awk '{ print $2 }' | sed 's/K//'`
        CURRENT_MEM_L=`pmap $T_PID | head -4 | tail -1 | awk '{ print $2 }' | sed 's/K//'`
        #CPU=`awk "BEGIN{ print $CPU + $CURRENT_CPU }"`
        CPU=`python3 -c "print (float($CPU) + float($CURRENT_CPU))"`
        CPU_L=`python3 -c "print (float($CPU_L) + float($CURRENT_CPU_L))"`
        MEM=`python3 -c "print (float($MEM) + float($CURRENT_MEM))"`
        MEM_L=`python3 -c "print (float($MEM_L) + float($CURRENT_MEM_L))"`
        sleep 0.1
        let COUNTER++
    done 
    TIME=$(date +%s)
    A_CPU=`python3 -c "print (float($CPU) / float($COUNTER))"`
    A_CPU=`python3 -c "print ('%.2f' % round($A_CPU, 2))"`
    A_CPU_L=`python3 -c "print (float($CPU_L) / float($COUNTER))"`
    A_CPU_L=`python3 -c "print ('%.2f' % round($A_CPU_L, 2))"`
    A_MEM=`python3 -c "print (float($MEM) / float($COUNTER))"`
    A_MEM=`python3 -c "print ('%.2f' % round($A_MEM, 2))"`
    A_MEM_L=`python3 -c "print (float($MEM_L) / float($COUNTER))"`
    A_MEM_L=`python3 -c "print ('%.2f' % round($A_MEM_L, 2))"`
    echo "$COUNT,pub_cpp,$TIME,$A_CPU,$A_MEM" >> ../data/resource-pubsub.csv
    echo "$COUNT,sub_cpp,$TIME,$A_CPU_L,$A_MEM_L" >> ../data/resource-pubsub.csv
    let COUNT++
done