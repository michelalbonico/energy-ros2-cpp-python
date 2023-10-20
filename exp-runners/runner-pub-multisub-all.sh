#!/bin/bash

_start=1
_end=20 # number of repetitions

source /opt/ros/humble/setup.bash
source ../install/setup.bash 
#echo "exec,label,timestamp,cpu,mem" > ../data/resource.csv

COUNT=744

# Each Algorithm
for _alg in pubsub pubsub_static teleop teleop_static
#for _alg in teleop
do
    # Each Language
    for _lang in py cpp
    #for _lang in cpp
    do
        # Multiple subscribers
        for _listeners in 1 10 20 30 40 
        #for _listeners in 30
        do
            # For the teleop, it stresses the CPU, therefore we must use 4 subscribers at max
            echo ""
            echo "Running $_alg in $_lang with $_listeners subscribers and $_end repetitions..."
            echo ""
            # Repeating
            for i in $(seq ${_start} ${_end})
            do
                # Killing possible remaining nodes
                pkill python3
                pkill talker
                pkill listener
                pkill movefb
                pkill turtlesim_node

                ./progress-bar.sh ${_start} ${_end} ${i}
            
                if [[  "$_alg" == "pubsub" ]]; then
                    if [[ "$_lang" == "py" ]]; then
                        # pubsub python code
                        pub="simple_pub_sub_py -a talker -i pub_py_$_listeners"
                        sub="simple_pub_sub_py -a listener -i sub_py_$_listeners"
                        _t_cmd="simple_pub_sub_py/talker"   
                        _l_cmd="simple_pub_sub_py/listener"
                    else
                        # pubsub cpp code
                        pub="simple_pub_sub_cpp -a talker -i pub_cpp_$_listeners"
                        sub="simple_pub_sub_cpp -a listener -i sub_cpp_$_listeners"
                        _t_cmd="simple_pub_sub_cpp/talker"
                        _l_cmd="simple_pub_sub_cpp/listener"
                    fi
                    _t_node="talker"
                    _l_node="listener"
                else
                    if [[  "$_alg" == "pubsub_static" ]]; then
                        if [[ "$_lang" == "cpp" ]]; then
                            # pubsub_static
                            pub="simple_pub_sub_cpp -a talker-static -i pub_static_cpp_$_listeners"
                            sub="simple_pub_sub_cpp -a listener -i sub_static_cpp_$_listeners"
                            _t_cmd="simple_pub_sub_cpp/talker-static"   
                            _l_cmd="simple_pub_sub_cpp/listener"
                        fi
                        _t_node="talker-static"
                        _l_node="listener"
                    else
                        # if [[  "$_listeners" -le 10 ]]; then ## only 2 listeners (otherwise there is resource exaustion)
                            # if [[  "$_listeners" -eq 10 ]]; then
                            #     _listeners=2
                            # fi
                            if [[  "$_alg" == "teleop_static" ]]; then
                                _t_node="movefb-static"
                                if [[ "$_lang" == "cpp" ]]; then
                                    # teleop-static cpp code
                                    pub="turtlesim_linear_cpp -a movefb-static -i teleop_pub_static_cpp_$_listeners"
                                    sub="my_turtlesim_cpp -a turtlesim_node -i teleop_sub_static_cpp_$_listeners"
                                    _t_cmd="turtlesim_linear_cpp/movefb-static"
                                    _l_cmd="my_turtlesim_cpp/turtlesim_node"
                                else 
                                    pub="turtlesim_linear_py -a movefb -i teleop_pub_py_$_listeners"
                                    sub="my_turtlesim -a turtlesim_node -i teleop_sub_py_$_listeners"
                                    _t_cmd="turtlesim_linear_py/movefb"
                                    _l_cmd="my_turtlesim/turtlesim_node"
                                    _t_node="movefb"
                                fi
                                _l_node="turtlesim_node"
                            else
                                if [[ "$_lang" == "py" ]]; then
                                    # teleop python code
                                    pub="turtlesim_linear_py -a movefb -i teleop_pub_py_$_listeners"
                                    sub="my_turtlesim -a turtlesim_node -i teleop_sub_py_$_listeners"
                                    _t_cmd="turtlesim_linear_py/movefb"
                                    _l_cmd="my_turtlesim/turtlesim_node"
                                else
                                    # teleop cpp code
                                    pub="turtlesim_linear_cpp -a movefb -i teleop_pub_cpp_$_listeners"
                                    sub="my_turtlesim_cpp -a turtlesim_node -i teleop_sub_cpp_$_listeners"
                                    _t_cmd="turtlesim_linear_cpp/movefb"
                                    _l_cmd="my_turtlesim_cpp/turtlesim_node"
                                fi
                                _t_node="movefb"
                                _l_node="turtlesim_node"
                            fi
                        # else
                        #     echo "out of resources"
                        #     echo ""
                        #     break
                        # fi
                    fi
                fi

                python3 ../energy-mon.py -p $pub &> /dev/null &
                TALKER_PID=$!
                sleep 0.5

                for ii in $(seq 1 ${_listeners}) # multiple listeners
                do
                    python3 ../energy-mon.py -p $sub &> /dev/null & 
                    LISTENER_PID=$!
                done

                # tcpdump -i lo -G 5 -W 1 -w capture.pcap &> pkgs.log
                # _packets=`cat pkgs.log | tail -3 | head -1 | awk '{ print $1 }'`

                sleep 1
                T_PID=`ps ax | grep $_t_cmd | head -1 | awk '{ print $1 }'`
                L_PID=`ps ax | grep $_l_cmd | head -1 | awk '{ print $1 }'`

                #pidstat -u -p $T_PID 1 20
                perf top -p $T_PID --sort comm,dso
                
                MEM=0
                CPU_L=0.0
                MEM_L=0
                A_CPU=0
                A_MEM=0
                A_CPU_L=0
                A_MEM_L=0
                while kill -0 $LISTENER_PID 2> /dev/null; do 
                    CURRENT_CPU=`ps -C $_t_node -o %cpu | tail -1 | grep [0-9]`
                    CURRENT_MEM=`pmap $T_PID | tail -1 | awk '{ print $2 }' | sed 's/K//'`
                    CURRENT_CPU_L=`ps -C $_l_node -o %cpu | tail -1 | grep [0-9]`
                    CURRENT_MEM_L=`pmap $L_PID | tail -1 | awk '{ print $2 }' | sed 's/K//'`
                    CPU=`python3 -c "print (float($CPU) + float($CURRENT_CPU))"`
                    MEM=`python3 -c "print (float($MEM) + float($CURRENT_MEM))"`
                    CPU_L=`python3 -c "print (float($CPU_L) + float($CURRENT_CPU_L))"`
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
                if [[  "$_alg" == "pubsub_static" || "$_alg" == "teleop_static" ]];
                then
                    _l_node=`echo "$_l_node-static"`
                fi
                echo "$COUNT,$_alg-$_t_node-$_lang-$_listeners,$TIME,$A_CPU,$A_MEM" >> ../data/resource.csv
                echo "$COUNT,$_alg-$_l_node-$_lang-$_listeners,$TIME,$A_CPU_L,$A_MEM_L" >> ../data/resource.csv
                let COUNT++
                sleep 10
            done # n repetitions
            sleep 10
        done # multiple sub
        sleep 10
    done # lang
    sleep 10
done # algorithm