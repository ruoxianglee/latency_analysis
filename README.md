# latency_analysis

## Worst-Case Latency Analysis of Message Synchronization in ROS 
This is a **ROS2 Humble** package providing the evaluation test about the worst-case latency analysis for our model of [ROS Approximate Time policy](https://github.com/ros2/message_filters/blob/master/include/message_filters/sync_policies/approximate_time.h).

## Package Structure
```
> tree .
|-- config
    |-- config_x.yaml                         # ROS2 parameters, i.e., test setting
|-- include         
    |-- helper.h                              # Some helper functions    
    |-- publisher.h                           # Message publishing node using timer. There is a delay time before publishing
    |-- subscriber_latency.h                    # Message subscription node for latency evaluation (invoke Synchronizer & record timing infomation)
|-- message_filters                           # Files in this directory should be put into `message_filters` package
    |-- sync_policies
        |-- approximate_time_model.h          # Our implementation for modeling Approximate Time policy
|-- msg
    |-- Timing.msg                            # Timing data structure, i.e., ROS 2 interface, for message timing trace
|-- periods                                   # Used for Publisher to save the period files
|-- scripts
    |-- bound_with_delay.py                   # Time disparity upper bound, i.e., $\overline{\delta}$, calculation
    |-- latency_calculation.py                # Passing latency & Reaction latency calculation
    |-- passing_latency_line_chart_plot.py    # Line chart plotting for passing latency
    |-- passing_latency_result.py             # Passing latency results generation (Figure 6)
    |-- reaction_latency_line_chart_plot.py   # Line chart plotting for reaction latency
    |-- reaction_latency_result.py            # Reaction latency results generation (Figure 7)
|-- src
    |-- pubx_exe.cpp                          # Publisher in a single-theaded executor. Each Publisher x publishes msg to topic 'topicx' with its own period
    |-- sub_exe.cpp                           # Synchronizer in a single-threaded executor
|-- prepare.sh                                # The script for automatically adding files and building packages
|-- run_evaluation.sh                         # The script for launching the experiments
```

## Before building

### Environment
- [Build ROS2 Humble in `ros2_humble` workspace](https://docs.ros.org/en/humble/Installation/Ubuntu-Development-Setup.html)
- Clone this project into your workspace (under 'src' directory)

### Files preparation

- Copy file in `message_filters/sync_policies` to `ros2_humble/src/ros2/message_filters/include/message_filters/sync_policies`
- Copy file `msg/Timing.msg` to `/ros2_humble/src/ros2/common_interfaces/std_msgs/msg/`
- Add the following line to file `/ros2_humble/src/ros2/common_interfaces/std_msgs/CMakeLists.txt`, just after `set(msg_files`
    ```sh
    "msg/Timing.msg"
    ```
- Modify content of file `/ros2_humble/src/ros2/common_interfaces/sensor_msgs/msg/JointState.msg` as follows:
    ```sh
    std_msgs/Timing timing   # This line should be added
    std_msgs/Header header

    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
    ```

## Build

### Build interface
 ```sh
cd ros2_humble
source ./install/setup.bash
colcon build --packages-select std_msgs --symlink-install
source ./install/setup.bash
colcon build --packages-select sensor_msgs --symlink-install
 ```

### Build `message_filters` package

 ```sh
cd ros2_humble
source ./install/setup.bash
colcon build --packages-select message_filters --symlink-install
 ```
 
### Build this project
```sh
cd ros2_humble
source ./install/setup.bash
colcon build --packages-select latency_analysis --symlink-install
```

## Run evaluation

```sh
cd ~/ros2_humble/src/latency_analysis
chmod +x run_delay_test.sh
./run_delay_test.sh
```

**Note**: You can modify the parameter `num_group` in line 6 of `run_delay_test.sh` to set the number of groups to be tested. The default value is set as 10 (which is 100 for the representive result in our paper) for shorter running times. Also for the same reason, the configuration parameter `num_published_set` (i.e., the number of published sets for each group) in configuration files is set as 20 (which is 5000 in our paper).

After finishing the evaluation, the results (in `results/evaluation_delay`) can be analyzed.

## Result analysis

### Passing latency
```sh
cd ~/ros2_humble/src/latency_analysis/scripts
python3 passing_latency_result.py
```

### Reaction latency
```sh
cd ~/ros2_humble/src/latency_analysis/scripts
python3 reaction_latency_result.py
```