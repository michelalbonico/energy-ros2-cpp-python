#!/bin/bash

_start=1
_end=20

source /opt/ros/humble/setup.bash
source install/setup.bash 

pkill python3
COUNT=0
exp_result="./exp_runners/experiments/cpp_py_ros2_action_standalone"
mkdir -p $exp_result


server_name='action_server'
client_name='action_client'


for i in $(seq ${_start} ${_end})
do
    run=0
    for interval in "0.05" "0.1" "0.25" "0.5" "1.0"; do
        for lang in "py" "cpp"; do
            for cli in $(seq 1 3); do
                pkill -9 -f python3
                pkill -9 -f action_server
                pkill -9 -f action_client
                sleep 1
                echo "Running: language ($lang), interval ($interval), cli ($cli)"
                d_folder="${exp_result}/run_${run}_repetition_${i}"
                echo $d_folder
                rm -Rf $d_folder
                mkdir -p $d_folder

                echo "exec,label,timestamp,cpu,mem" > $d_folder/server-cpu-mem.csv
                echo "exec,label,timestamp,cpu,mem" > $d_folder/client-cpu-mem.csv

                echo "exec,label,timestamp,cpu" > $d_folder/server-cpu.csv
                echo "exec,label,timestamp,cpu" > $d_folder/client-cpu.csv

                python3 mon-rapl.py -p action_tutorials_$lang -a action_server -i server -f $interval -t 65 &> $d_folder/server-$lang-$interval-$cli.log &
                TALKER_PID=$!

                for i in $(seq 1 ${cli}); do
                    echo "here"
                    python3 mon-rapl.py -p action_tutorials_$lang -a action_client -i client -f $interval -t 60 &> $d_folder/client-$lang-$interval-$cli.log &
                    LISTENER_PID=$!
                done

                sleep 1

                CPU=0.0
                MEM=0
                CPU_L=0.0
                MEM_L=0

                TALKER_PID=`ps -C $server_name | tail -1 | grep [0-9] | awk '{ print $1}'`
                echo "Talker PID: $TALKER_PID"
                LISTENER_PID=`ps -C $client_name | tail -1 | grep [0-9] | awk '{ print $1}'`
                echo "Talker PID: $LISTENER_PID"

                while kill -0 $TALKER_PID 2> /dev/null; do
                    TIME=$(date +%s)
                    CURRENT_CPU_PS=`ps -C action_server -o %cpu | tail -1 | grep [0-9]`
                    CURRENT_CPU=`top -bn1 | grep action_server | tail -1 | awk '{print $9}'`
                    CURRENT_CPU=`echo $CURRENT_CPU | sed 's/,/./g'`
                    CURRENT_CPU_L_PS=`ps -C action_client -o %cpu | tail -1 | grep [0-9]`
                    CURRENT_CPU_L=`top -bn1 | grep action_client | tail -1 | awk '{print $9}'` 
                    CURRENT_CPU_L=`echo $CURRENT_CPU_L | sed 's/,/./g'`
                    CURRENT_MEM=`pmap $TALKER_PID | head -3 | tail -1 | awk '{ print $2 }' | sed 's/K//'`
                    CURRENT_MEM_L=`pmap $LISTENER_PID | head -4 | tail -1 | awk '{ print $2 }' | sed 's/K//'`
                    CPU=`python3 -c "print (float($CURRENT_CPU))"`
                    CPU_L=`python3 -c "print (float($CURRENT_CPU_L))"`
                    MEM=`python3 -c "print (float($CURRENT_MEM))"`
                    MEM_L=`python3 -c "print (float($CURRENT_MEM_L))"`
                    
                    echo "$COUNT,fibonacci_action_server,$TIME,$CPU,$MEM" >> $d_folder/server-cpu-mem.csv
                    echo "$COUNT,fibonacci_action_client,$TIME,$CPU_L,$MEM_L" >> $d_folder/client-cpu-mem.csv

                    echo "$COUNT,fibonacci_action_server,$TIME,$CURRENT_CPU_PS" >> $d_folder/server-cpu.csv
                    echo "$COUNT,fibonacci_action_client,$TIME,$CURRENT_CPU_PS" >> $d_folder/client-cpu.csv
                    
                    #
                    sleep 0.1
                done 
                cp -f energy-* $d_folder
                rm -Rf energy-*
                let COUNT++
                let run++
            done
        done
    done
done