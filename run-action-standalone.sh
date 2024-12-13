#!/bin/bash

_start=1
_end=20

source /opt/ros/humble/setup.bash
source install/setup.bash 

server_name='action_server'
client_name='action_client'

pkill python3
COUNT=0
exp_result="./exp_runners/experiments/cpp_py_ros2_action_standalone_multi_log"
mkdir -p $exp_result

python3 exp_runners/standalone/read_runtable.py exp_runners/standalone/action_runtable.csv > action_remain_runs.txt

run_name=''
package_name=''
msg_interval=''
lang=''
timeout=''
clients=''

file="action_remain_runs.txt"

if [[ -f "$file" ]]; then
  while IFS= read -r line; do
    read run_name todo package_name interval lang timeout cli <<< "$line"
    # Killing remaining processes
    pkill -9 -f python3
    pkill -9 -f $server_name
    pkill -9 -f $client_name
    pkill -9 -f powerjoular
    sleep 1
    echo "Running: language ($lang), interval ($interval), cli ($cli)"
    d_folder="${exp_result}/${run_name}"
    echo $d_folder
    rm -Rf $d_folder
    mkdir -p $d_folder

    echo "exec,label,timestamp,cpu,mem" > $d_folder/server-cpu-mem.csv
    echo "exec,label,timestamp,cpu,mem" > $d_folder/client-cpu-mem.csv

    echo "exec,label,timestamp,cpu" > $d_folder/server-cpu.csv
    echo "exec,label,timestamp,cpu" > $d_folder/client-cpu.csv

    python3 mon-rapl.py -p ${package_name}_${lang} -a $server_name -i server -f $interval -t $timeout -r 1 &> $d_folder/server-$lang-$interval-$cli.log &
    echo "python3 mon-rapl.py -p ${package_name}_${lang} -a $server_name -i server -f $interval -t $timeout -r 1 &> $d_folder/server-$lang-$interval-$cli.log &"
    SERVER_PID=$!
    echo "Started mon-rapl.py with PID $SERVER_PID"

    for i in $(seq 1 ${cli}); do
        echo "Adding client..."
        rapl="-r 0"
        if [ $i -eq $cli ]; then
            rapl="-r 1"
        fi
        python3 mon-rapl.py -p ${package_name}_${lang} -a $client_name -i client -f $interval -t $timeout $rapl &> $d_folder/client-$lang-$interval-$cli-$i.log &
        echo "python3 mon-rapl.py -p ${package_name}_${lang} -a $client_name -i client -f $interval -t $timeout $rapl &> $d_folder/client-$lang-$interval-$cli-$i.log &"
        CLIENT_PID=$!
    done

    sleep 1

    TALKER_PID=`ps -C $server_name | tail -1 | grep [0-9] | awk '{ print $1}'`
    echo "Server PID: $TALKER_PID"
    LISTENER_PID=`ps -C $client_name | tail -1 | grep [0-9] | awk '{ print $1}'`
    echo "Client PID: $LISTENER_PID"


    echo "Running PowerJoular"
    /usr/bin/powerjoular -p $TALKER_PID -f 'energy-server-powerjoular.csv' &
    /usr/bin/powerjoular -p $LISTENER_PID -f 'energy-client-powerjoular.csv' &

    CPU=0.0
    MEM=0
    CPU_L=0.0
    MEM_L=0

    spent_time=0

    while kill -0 $SERVER_PID 2> /dev/null || kill -0 $CLIENT_PID 2> /dev/null || "$(echo "$spent_time < $timeout" | bc)" -eq 1; do
        TIME=$(date +%s)
        CURRENT_CPU_PS=`ps -C $server_name -o %cpu | tail -1 | grep [0-9]`
        CURRENT_CPU=`top -bn1 | grep $server_name | tail -1 | awk '{print $9}'`
        CURRENT_CPU=`echo $CURRENT_CPU | sed 's/,/./g'`
        CURRENT_CPU_L_PS=`ps -C $client_name -o %cpu | tail -1 | grep [0-9]`
        CURRENT_CPU_L=`top -bn1 | grep $client_name | tail -1 | awk '{print $9}'` 
        CURRENT_CPU_L=`echo $CURRENT_CPU_L | sed 's/,/./g'`
        CURRENT_MEM=`pmap $SERVER_PID | head -3 | tail -1 | awk '{ print $2 }' | sed 's/K//'`
        CURRENT_MEM_L=`pmap $CLIENT_PID | head -4 | tail -1 | awk '{ print $2 }' | sed 's/K//'`
        CPU=`python3 -c "print (float($CURRENT_CPU))"`
        CPU_L=`python3 -c "print (float($CURRENT_CPU_L))"`
        MEM=`python3 -c "print (float($CURRENT_MEM))"`
        MEM_L=`python3 -c "print (float($CURRENT_MEM_L))"`
        
        echo "$COUNT,action_server,$TIME,$CPU,$MEM" >> $d_folder/server-cpu-mem.csv
        echo "$COUNT,action_client,$TIME,$CPU_L,$MEM_L" >> $d_folder/client-cpu-mem.csv

        echo "$COUNT,action_server,$TIME,$CURRENT_CPU_PS" >> $d_folder/server-cpu.csv
        echo "$COUNT,action_client,$TIME,$CURRENT_CPU_L_PS" >> $d_folder/client-cpu.csv
        sleep 0.1
        spent_time=$(echo "$spent_time + 0.5" | bc)
        if [ "$(echo "$spent_time > $timeout" | bc)" -eq 1 ]; then
          echo "Timeout!"
          pkill -9 -f action_server
          pkill -9 -f action_client
          pkill -9 -f powerjoular
        fi
    done 
    echo "Stopped"
    if [ "$(echo "$spent_time > $timeout" | bc)" -eq 1 ]; then
        echo "Timeout!"
    fi
    
    cp -f energy-* $d_folder
    rm -Rf energy-*
    let COUNT++

    # Update the run to DONE
    python3 exp_runners/standalone/update_runtable.py exp_runners/standalone/action_runtable.csv $run_name

    echo "Sleeping..."
    sleep 10
  done < "$file"
else
  echo "File $file not found!"
fi