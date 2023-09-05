import numpy as np
import matplotlib.pyplot as plt
import bound_with_delay as bound_lib
from matplotlib.ticker import MaxNLocator
import time
import latency_calculation as plc
import passing_latency_line_chart_plot as lcp

channel_index = 0 # Configure to calculate upper bound and observation for different channels [0,5]

#############################################
# Get results of evaluation for changed period
#############################################
# results = []
# period_lower = [10, 20, 30, 40, 50]
# for period_lower in period_lower:
#     path="../results/evaluation/varied_periods/varied_period_lower_"+str(period_lower)

#     avg_upper_bound1, avg_upper_bound2, avg_observed = plc.calculate_average_passing_latency(path, 6, channel_index)
#     results.append([avg_upper_bound1 * 10, avg_upper_bound2 * 10, avg_observed * 10])

# upper_bound1_array=[]
# upper_bound2_array=[]
# observed_array=[]
# for k in range(5):
#     upper_bound1_array.append(results[k][0])
#     upper_bound2_array.append(results[k][1])
#     observed_array.append(results[k][2])

# print("#### Passing Latency: Varied Period")
# print("Upper Bound 1:", upper_bound1_array)
# print("Upper Bound 2:", upper_bound2_array)
# print("Observed: ", observed_array)

# lcp.line_chart_plot_varied_period(results)

#############################################
# Get results of evaluation for changed delay
#############################################
results = []
delay_lower = [0, 10, 20, 30, 40]
for delay_lower in delay_lower:
    if delay_lower == 0:
        path="../results/evaluation/varied_delay/random_nodelay"
    else:
        path="../results/evaluation/varied_delay/random_delay_"+str(delay_lower)

    avg_upper_bound1, avg_upper_bound2, avg_observed = plc.calculate_average_passing_latency(path, 6, channel_index)
    results.append([avg_upper_bound1 * 10, avg_upper_bound2 * 10, avg_observed * 10])

upper_bound1_array=[]
upper_bound2_array=[]
observed_array=[]
for k in range(5):
    upper_bound1_array.append(results[k][0])
    upper_bound2_array.append(results[k][1])
    observed_array.append(results[k][2])

print("#### Passing Latency: Random Delay")
print("Upper Bound 1:", upper_bound1_array)
print("Upper Bound 2:", upper_bound2_array)
print("Observed: ", observed_array)

lcp.line_chart_plot_varied_delay(results)

#############################################
# Get results of evaluation for changed channels
#############################################
results = []
channels = [3, 4, 5, 6, 7, 8, 9]
for channel_num in channels:
    path="../results/evaluation/channel_num/channel_" + str(channel_num)

    avg_upper_bound1, avg_upper_bound2, avg_observed = plc.calculate_average_passing_latency(path, channel_num, channel_index)
    results.append([avg_upper_bound1 * 10, avg_upper_bound2 * 10, avg_observed * 10])

upper_bound1_array=[]
upper_bound2_array=[]
observed_array=[]
for k in range(7):
    upper_bound1_array.append(results[k][0])
    upper_bound2_array.append(results[k][1])
    observed_array.append(results[k][2])

print("#### Passing Latency: Number of Channels")
print("Upper Bound 1:", upper_bound1_array)
print("Upper Bound 2:", upper_bound2_array)
print("Observed: ", observed_array)

lcp.line_chart_plot_different_channels(results)

#############################################
# Get results of evaluation for changed period ratio
#############################################
results = []
factors = [0, 2, 4, 6, 8]
for factor in factors:
    path="../results/evaluation/varied_period_factor/varied_period_factor_1."+str(factor)

    avg_upper_bound1, avg_upper_bound2, avg_observed = plc.calculate_average_passing_latency(path, 6, channel_index)
    results.append([avg_upper_bound1 * 10, avg_upper_bound2 * 10, avg_observed * 10])

upper_bound1_array=[]
upper_bound2_array=[]
observed_array=[]
for k in range(5):
    upper_bound1_array.append(results[k][0])
    upper_bound2_array.append(results[k][1])
    observed_array.append(results[k][2])

print("#### Passing Latency: Period Ratio")
print("Upper Bound 1:", upper_bound1_array)
print("Upper Bound 2:", upper_bound2_array)
print("Observed: ", observed_array)

lcp.line_chart_plot_period_factor(results)
#############################################
# Get results of evaluation for 6 channels
#############################################
results = []
for channel in range(6):
    path="../results/evaluation/channel_num/channel_6"

    avg_upper_bound1, avg_upper_bound2, avg_observed = plc.calculate_average_passing_latency(path, 6, channel)
    results.append([avg_upper_bound1 * 10, avg_upper_bound2 * 10, avg_observed * 10])

upper_bound1_array=[]
upper_bound2_array=[]
observed_array=[]
for k in range(6):
    upper_bound1_array.append(results[k][0])
    upper_bound2_array.append(results[k][1])
    observed_array.append(results[k][2])

print("#### Passing Latency: Index of Channel")
print("Upper Bound 1:", upper_bound1_array)
print("Upper Bound 2:", upper_bound2_array)
print("Observed: ", observed_array)

lcp.line_chart_plot_six_channels(results)