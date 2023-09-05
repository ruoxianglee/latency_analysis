import numpy as np
import matplotlib.pyplot as plt
import bound_with_delay as bound_lib
from matplotlib.ticker import MaxNLocator
import time

# Calculate the average upper bound 1/2 and average observed passing latency
# based on a file data.
# @param: channel specifies the corresponding channel for calculation.
def calculate_average_passing_latency(path, channel_nums, channel):
    average_upper_bound_1=0
    average_upper_bound_2=0
    average_observed=0
    count=0

    # Open the timestamp file for reading
    with open(path + "/timestamp_mdl.txt", "r") as f:
        for line in f:
            count+=1
            
            # Split the line into a list of floats
            values = [float(x) for x in line.split()]

            best_periods = []
            for index in range(channel_nums):
                best_periods.append(values[index])

            worst_periods = []
            for index in range(channel_nums):
                worst_periods.append(values[channel_nums + index])

            best_delays = []
            for index in range(channel_nums):
                best_delays.append(values[2*channel_nums + index])

            worst_delays = []
            for index in range(channel_nums):
                worst_delays.append(values[3*channel_nums + index])

            observed = []
            for index in range(channel_nums):
                observed.append(values[4*channel_nums + index])

            average_observed+=observed[channel]

            # Calculate time disparity
            time_disparity = bound_lib.cal_ros_bound(worst_periods)
            
            # Calculate Upper Bound 1
            max_sum = 0
            for idx in range(channel_nums):
                val = worst_periods[idx] + worst_delays[idx]
                if val > max_sum:
                    max_sum = val

            # print("Maximum sum for upper bound 1: ", max_sum)
            upper_bound_1 = (max_sum + time_disparity - best_delays[channel])
            average_upper_bound_1 += upper_bound_1

            # Calculate Upper Bound 2
            max_sum = 0

            ## Case 1: best_period < time_disparity
            indexes = [i for i, val in enumerate(best_periods) if val < time_disparity]
            # print("Reduced indexes for case 1", indexes)
            for idx in indexes:
                val = worst_periods[idx] + worst_delays[idx] 
                if val > max_sum:
                    max_sum = val

            ## Case 2: time_disparity < best_period < 2*time_disparity
            indexes = []
            for ind, val in enumerate(best_periods):
                if val >= time_disparity and val < 2 * time_disparity:
                    indexes.append(ind)
            # print("Reduced indexes for case 2", indexes)
            for idx in indexes:
                val = worst_periods[idx] + worst_delays[idx] + time_disparity - best_periods[idx]
                if val > max_sum:
                    max_sum = val                  

            # print("Maximum sum for upper bound 2: ", max_sum)
            upper_bound_2 = (max_sum + time_disparity - best_delays[channel])
            average_upper_bound_2 += upper_bound_2

            if upper_bound_2 < observed[channel]:
                print(best_periods)
                print(worst_periods)
                print(best_delays)
                print(worst_delays)
                print(observed)
                print(time_disparity)
                print(upper_bound_2)
                print(observed[channel])
                print("WRONG UPPER BOUND 2!!!")
                break
    
    average_upper_bound_1 /= count
    average_upper_bound_2 /= count
    average_observed /= count

    return average_upper_bound_1, average_upper_bound_2, average_observed

def calculate_average_reaction_latency(path, channel_nums, channel):
    average_upper_bound=0
    average_observed=0
    count=0

    # Open the timestamp file for reading
    with open(path + "/timestamp_mdl.txt", "r") as f:
        for line in f:
            count+=1
            
            # Split the line into a list of floats
            values = [float(x) for x in line.split()]

            best_periods = []
            for index in range(channel_nums):
                best_periods.append(values[index])

            worst_periods = []
            for index in range(channel_nums):
                worst_periods.append(values[channel_nums + index])

            best_delays = []
            for index in range(channel_nums):
                best_delays.append(values[2*channel_nums + index])

            worst_delays = []
            for index in range(channel_nums):
                worst_delays.append(values[3*channel_nums + index])

            observed = []
            for index in range(channel_nums):
                observed.append(values[5*channel_nums + index])

            average_observed+=observed[channel]

            # Calculate time disparity
            time_disparity = bound_lib.cal_ros_bound(worst_periods)
            
            # Calculate Queuing Latency Upper Bound
            max_worst_period = 0
            max_worst_period_index = 0
            for idx in range(channel_nums):
                val = worst_periods[idx]
                if val > max_worst_period:
                    max_worst_period_index = idx
                    max_worst_period = val
            Gamma=2*time_disparity + max_worst_period
            upper_bound_queuing_latency = Gamma + worst_delays[channel] - best_delays[channel]

            # Calculate Passing Latency Upper Bound
            max_sum = 0

            ## Case 1: best_period < time_disparity
            indexes = [i for i, val in enumerate(best_periods) if val < time_disparity]
            # print("Reduced indexes for case 1", indexes)
            for idx in indexes:
                val = worst_periods[idx] + worst_delays[idx] 
                if val > max_sum:
                    max_sum = val

            ## Case 2: time_disparity < best_period < 2*time_disparity
            indexes = []
            for ind, val in enumerate(best_periods):
                if val >= time_disparity and val < 2 * time_disparity:
                    indexes.append(ind)
            # print("Reduced indexes for case 2", indexes)
            for idx in indexes:
                val = worst_periods[idx] + worst_delays[idx] + time_disparity - best_periods[idx]
                if val > max_sum:
                    max_sum = val                  

            # print("Maximum sum for upper bound 2: ", max_sum)
            upper_bound_passing_latency = (max_sum + time_disparity - best_delays[channel])

            average_upper_bound += (upper_bound_queuing_latency + upper_bound_passing_latency)
    
    average_upper_bound /= count
    average_observed /= count

    return average_upper_bound, average_observed


# The following two functions are not used.
def calculate_average_timestamp_difference(path, channel_nums, channel):
    average_upper_bound=0
    average_observed=0
    count=0

    # Open the timestamp file for reading
    with open(path + "/timestamp_mdl.txt", "r") as f:
        for line in f:
            count+=1
            
            # Split the line into a list of floats
            values = [float(x) for x in line.split()]

            best_periods = []
            for index in range(channel_nums):
                best_periods.append(values[index])

            worst_periods = []
            for index in range(channel_nums):
                worst_periods.append(values[channel_nums + index])

            best_delays = []
            for index in range(channel_nums):
                best_delays.append(values[2*channel_nums + index])

            worst_delays = []
            for index in range(channel_nums):
                worst_delays.append(values[3*channel_nums + index])

            observed = []
            for index in range(channel_nums):
                observed.append(values[7*channel_nums + index])

            average_observed+=observed[channel]

            # Calculate time disparity
            time_disparity = bound_lib.cal_ros_bound(worst_periods)
            
            # Calculate Timestamp Difference Upper Bound
            max_worst_period = 0
            for idx in range(channel_nums):
                val = worst_periods[idx]
                if val > max_worst_period:
                    max_worst_period = val
            upper_bound=2*time_disparity + max_worst_period
            
            average_upper_bound += upper_bound

            # if channel == 0:
            print("Time disparity: ", time_disparity, "  Upper bound: ", upper_bound, "  Observed: ", observed[channel])
    
    average_upper_bound /= count
    average_observed /= count

    return average_upper_bound, average_observed

def calculate_average_queuing_latency(path, channel_nums, channel):
    average_upper_bound=0
    average_observed=0
    count=0

    # Open the timestamp file for reading
    with open(path + "/timestamp_mdl.txt", "r") as f:
        for line in f:
            count+=1
            
            # Split the line into a list of floats
            values = [float(x) for x in line.split()]

            best_periods = []
            for index in range(channel_nums):
                best_periods.append(values[index])

            worst_periods = []
            for index in range(channel_nums):
                worst_periods.append(values[channel_nums + index])

            best_delays = []
            for index in range(channel_nums):
                best_delays.append(values[2*channel_nums + index])

            worst_delays = []
            for index in range(channel_nums):
                worst_delays.append(values[3*channel_nums + index])

            observed = []
            for index in range(channel_nums):
                observed.append(values[4*channel_nums + index])

            average_observed+=observed[channel]

            # Calculate time disparity
            time_disparity = bound_lib.cal_ros_bound(worst_periods)
            
            # Calculate Upper Bound
            max_worst_period = 0
            for idx in range(channel_nums):
                val = worst_periods[idx]
                if val > max_worst_period:
                    max_worst_period = val
            Gamma=2*time_disparity + max_worst_period
            upper_bound = Gamma - best_periods[channel] + worst_delays[channel] - best_delays[channel]
            
            average_upper_bound += upper_bound

            if channel == 0:
                print("Time disparity: ", time_disparity, "  Upper bound: ", upper_bound, "  Observed: ", observed[channel])

    average_upper_bound /= count
    average_observed /= count

    return average_upper_bound, average_observed
