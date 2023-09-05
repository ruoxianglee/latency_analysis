#!/bin/bash

source ../../install/setup.bash

# Predefined Parameter
num_group=5
periods_path="/home/"$HOSTNAME"/ros2_humble/src/latency_analysis/periods"

# Define an array with 9 values to launch 9 executables
cmds=( "pub1_exe" "pub2_exe" "pub3_exe" "pub4_exe" "pub5_exe" "pub6_exe" "pub7_exe" "pub8_exe" "pub9_exe" )

channel_num=${#cmds[@]}

for (( k=0; k<num_group; k++))
do  
    ##############################################
    # Evaluation with varied timestamp separation 
    ##############################################
    channel_num=6
    period_lower_limit=50
    for (( j=0; j<5; j++ ))
    do
        echo "-------------------Evaluation for varied timestamp separation: $j, group: $k.-------------------"
        factor=$((2 * j))
        result_path="/home/"$HOSTNAME"/ros2_humble/src/latency_analysis/results/evaluation/varied_period_factor/varied_period_factor_1.$factor"

        # Launch each executable in a loop and save the PID to array
        config_index=$((2 + j))
        for (( i=0; i<channel_num; i++ ))
        do
            ros2 run latency_analysis ${cmds[i]} $period_lower_limit $periods_path --ros-args --params-file /home/"$HOSTNAME"/ros2_humble/src/latency_analysis/config/config_$config_index.yaml &
        done

        # Sleep for 2 seconds
        sleep 2

        ros2 run latency_analysis sub_exe $channel_num $result_path $periods_path --ros-args --params-file /home/"$HOSTNAME"/ros2_humble/src/latency_analysis/config/config_$config_index.yaml

        # Sleep for 2 seconds
        sleep 2

        for (( i=0; i<channel_num; i++ ))
        do
            killall ${cmds[i]}
        done

        # Wait for all executables to complete
        wait
    done

    ##############################################
    # Evaluation with different number of channels
    ##############################################
    period_lower_limit=50
    for (( j=3; j<10; j++ ))
    do
        echo "-------------------Evaluation for different number of channels: $j, group: $k.-------------------"
        channel_num=$j
        result_path="/home/"$HOSTNAME"/ros2_humble/src/latency_analysis/results/evaluation/channel_num/channel_$channel_num"

        # Launch each executable in a loop and save the PID to array
        for (( i=0; i<channel_num; i++ ))
        do
            ros2 run latency_analysis ${cmds[i]} $period_lower_limit $periods_path --ros-args --params-file /home/"$HOSTNAME"/ros2_humble/src/latency_analysis/config/config_1.yaml &
        done

        # Sleep for 2 seconds
        sleep 2

        ros2 run latency_analysis sub_exe $channel_num $result_path $periods_path --ros-args --params-file /home/"$HOSTNAME"/ros2_humble/src/latency_analysis/config/config_1.yaml

        # Sleep for 2 seconds
        sleep 2

        for (( i=0; i<channel_num; i++ ))
        do
            killall ${cmds[i]}
        done

        # Wait for all executables to complete
        wait
    done
    
    ##############################################
    # Evaluation with varied delay
    ##############################################
    channel_num=6
    period_lower_limit=50
    for (( j=1; j<6; j++ ))
    do
        echo "-------------------Evaluation for varied delay: $j, group: $k.-------------------"
        delay_upper=$((10 * j))
        result_path="/home/"$HOSTNAME"/ros2_humble/src/latency_analysis/results/evaluation/varied_delay/random_delay_$delay_upper"

        if [ $j -eq 5 ]
        then
            result_path="/home/"$HOSTNAME"/ros2_humble/src/latency_analysis/results/evaluation/varied_delay/random_nodelay"
        fi

        # Launch each executable in a loop and save the PID to array
        config_index=$((6 + j))
        for (( i=0; i<channel_num; i++ ))
        do
            ros2 run latency_analysis ${cmds[i]} $period_lower_limit $periods_path --ros-args --params-file /home/"$HOSTNAME"/ros2_humble/src/latency_analysis/config/config_$config_index.yaml &
        done

        # Sleep for 2 seconds
        sleep 2

        ros2 run latency_analysis sub_exe $channel_num $result_path $periods_path --ros-args --params-file /home/"$HOSTNAME"/ros2_humble/src/latency_analysis/config/config_$config_index.yaml

        # Sleep for 2 seconds
        sleep 2

        for (( i=0; i<channel_num; i++ ))
        do
            killall ${cmds[i]}
        done

        # Wait for all executables to complete
        wait
    done

done
wait

date

cd scripts
python3 passing_latency_result.py
python3 reaction_latency_result.py

exit
