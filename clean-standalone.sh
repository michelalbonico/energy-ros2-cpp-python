folder="exp_runners/experiments/cpp_py_ros2_pub_sub_0_1"

new_run_folder="exp_runners/experiments/cpp_py_ros2_pub_sub_standalone_cpp_1"

new_repetitions=`ls $new_run_folder`
for new_repetition in $new_repetitions; do
    echo "Cleaning $new_repetition"
    echo "Removing old $repetition"
#     rm -Rf $folder/$repetition

    echo "copying new files"
    sudo cp -Rf $new_run_folder/* $folder/
    sudo chown -R michel:michel $folder
    sudo chmod -R 755 $folder

    dest_folder="$folder/$new_repetition"
    if test -d $dest_folder; then
        echo "Removing energy files that are not used"
        rm -f $dest_folder/energy-client.csv
        rm -f $dest_folder/energy-server.csv
        rm -f $dest_folder/energy-client-powerjoular.csv
        rm -f $dest_folder/energy-server-powerjoular.csv
        powerjoular_server_file=`ls $dest_folder/ | grep powerjoular | grep server | tail -1`
        file_number=$(echo "$powerjoular_server_file" | awk -F'-' '{print $(NF)}')
        echo "Moving $powerjoular_server_file with number $file_number"
        mv $dest_folder/$powerjoular_server_file $dest_folder/energy-server-$file_number
        powerjoular_client_file=`ls $dest_folder/ | grep powerjoular | grep client | tail -1`
        client_file_number=$(echo "$powerjoular_client_file" | awk -F'-' '{print $(NF)}')
        echo "Moving $powerjoular_client_file with number $client_file_number"
        mv $dest_folder/$powerjoular_client_file $dest_folder/energy-client-$client_file_number
        echo "Moving CPU and Mem files"
        mv $dest_folder/client-cpu-mem.csv $dest_folder/cpu-mem-client.csv
        mv $dest_folder/server-cpu-mem.csv $dest_folder/cpu-mem-server.csv
    fi
done

# Remove old folders
# new_run_folder="exp_runners/experiments/cpp_py_ros2_action_standalone_low_frequency_cpp"

# repetitions=`ls $new_run_folder`
# for repetition in $repetitions; do
#     echo "Removing $repetition"
#     rm -Rf $folder/$repetition
# done

# copying new files
# sudo cp -Rf $new_run_folder/* $folder/
# sudo chown -R michel:michel $folder
# sudo chmod -R 755 $folder