#!/bin/bash


for i in {1..100}
do
    # python3 energy-mon.py -c src/simple_pub_sub_py/simple_pub_sub_py/minimal_publisher.py -i pub_py > ./talker.log &
    # TALKER_PID=$!
    # python3 energy-mon.py -c src/simple_pub_sub_py/simple_pub_sub_py/minimal_subscriber.py -i sub_py > ./listener.log & 
    # LISTENER_PID=$!

    # python3 energy-mon.py -c install/simple_pub_sub_cpp/lib/simple_pub_sub_cpp/talker -i pub_cpp -l cpp > ./talker.log &
    # TALKER_PID=$!
    # python3 energy-mon.py -c install/simple_pub_sub_cpp/lib/simple_pub_sub_cpp/listener -i sub_cpp -l cpp > ./listener.log & 
    # LISTENER_PID=$!

    python3 energy-mon.py -p simple_pub_sub_cpp -a talker -i pub_cpp > ./talker.log &
    TALKER_PID=$!
    python3 energy-mon.py -p simple_pub_sub_cpp -a listener -i sub_cpp > ./listener.log & 
    LISTENER_PID=$!

    echo "label,timestamp,cpu,mem" > resource.csv
    sleep 1
    timestamp=$(date +%s)
    mem=`pmap $TALKER_PID | grep total | awk '{ print $2 }' | sed 's/K//'`
    mem1=`pmap $LISTENER_PID | grep total | awk '{ print $2 }' | sed 's/K//'`
    cpu=`top -b -n 10 -d 0.2 -p $TALKER_PID | tail -1 | awk '{print $9}' | tr , .`
    cpu1=`top -b -n 10 -d 0.2 -p $LISTENER_PID | tail -1 | awk '{print $9}' | tr , .`
    
    echo "talker_cpp,$timestamp,$cpu,$mem" >> resource.csv
    echo "listener_cpp,$timestamp,$cpu1,$mem1" >> resource.csv

    #echo ">>>>>>" >> talker-res.log
    #pidstat 30 -ru -p $TALKER_PID >> talker-res.log &
    #echo ">>>>>>" >> listener-res.log
    #pidstat 30 -ru -p $LISTENER_PID >> listener-res.log &
    while kill -0 $LISTENER_PID 2> /dev/null; do
        sleep 1
    done 
done