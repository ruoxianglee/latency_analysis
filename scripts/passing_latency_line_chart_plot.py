import numpy as np
import matplotlib.pyplot as plt
import bound_with_delay as bound_lib
from matplotlib.ticker import MaxNLocator
import time
import latency_calculation as blc

def line_chart_plot_different_channels(results):
    ## Line Chart
    x = np.array(['3', '4', '5', '6', '7', '8', '9'])

    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound1_array=[]
    upper_bound2_array=[]
    observed_array=[]
    for k in range(7):
        upper_bound1_array.append(results[k][0])
        upper_bound2_array.append(results[k][1])
        observed_array.append(results[k][2])
    
    print(upper_bound1_array)
    print(upper_bound2_array)
    print(observed_array)
    # return
    ax.plot(x, upper_bound1_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound 1')
    # ax.scatter(x, upper_bound1_array, color="red", s=100)  # Add dots to represent data points

    # Plot the upper bound line
    ax.plot(x, upper_bound2_array, marker="v", color="#f6b57b", markersize=12, linewidth=2, label='Upper Bound 2')
    # ax.scatter(x, upper_bound2_array, color="red", s=100)  # Add dots to represent data points

    # Plot the observed line
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # Set the axis labels and title
    ax.set_xlabel('Number of Channels', fontsize=20)
    ax.set_ylabel("Passing Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # Add a grid to the chart
    ax.grid(axis="y", linestyle='--', alpha=0.7)

    # Customize the tick font sizes
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('./passing_latency_different_channels_1.svg', format='svg')

    # Show the plot
    # plt.show()


def line_chart_plot_six_channels(results):
    ## Line Chart
    x = np.array(['1', '2', '3', '4', '5', '6'])

    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound1_array=[]
    upper_bound2_array=[]
    observed_array=[]
    for k in range(6):
        upper_bound1_array.append(results[k][0])
        upper_bound2_array.append(results[k][1])
        observed_array.append(results[k][2])
    
    print(upper_bound1_array)
    print(upper_bound2_array)
    print(observed_array)
    # return
    ax.plot(x, upper_bound1_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound 1')
    # ax.scatter(x, upper_bound1_array, color="red", s=100)  # Add dots to represent data points

    # Plot the upper bound line
    ax.plot(x, upper_bound2_array, marker="v", color="#f6b57b", markersize=12, linewidth=2, label='Upper Bound 2')
    # ax.scatter(x, upper_bound2_array, color="red", s=100)  # Add dots to represent data points

    # Plot the observed line
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # Set the axis labels and title
    ax.set_xlabel('Index of Channel', fontsize=20)
    ax.set_ylabel("Passing Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # Add a grid to the chart
    ax.grid(axis="y", linestyle='--', alpha=0.7)

    # Customize the tick font sizes
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')    
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('./passing_latency_six_channels.svg', format='svg')

    # Show the plot
    # plt.show()

def line_chart_plot_varied_delay(results):
    ## Line Chart
    x = np.array(['0', '[0,10]', '[0,20]', '[0,30]', '[0,40]'])

    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound1_array=[]
    upper_bound2_array=[]
    observed_array=[]
    for k in range(5):
        upper_bound1_array.append(results[k][0])
        upper_bound2_array.append(results[k][1])
        observed_array.append(results[k][2])
    
    print(upper_bound1_array)
    print(upper_bound2_array)
    print(observed_array)
    # return
    ax.plot(x, upper_bound1_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound 1')
    # ax.scatter(x, upper_bound1_array, color="red", s=100)  # Add dots to represent data points

    # Plot the upper bound line
    ax.plot(x, upper_bound2_array, marker="v", color="#f6b57b", markersize=12, linewidth=2, label='Upper Bound 2')
    # ax.scatter(x, upper_bound2_array, color="red", s=100)  # Add dots to represent data points

    # Plot the observed line
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # Set the axis labels and title
    ax.set_xlabel('Random Delay (ms)', fontsize=20)
    ax.set_ylabel("Passing Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # Add a grid to the chart
    ax.grid(axis="y", linestyle='--', alpha=0.7)

    # Customize the tick font sizes
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')    
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('./passing_latency_varied_delay_1.svg', format='svg')

    # Show the plot
    # plt.show()    

def line_chart_plot_period_factor(results):
    ## Line Chart
    x = np.array(['1.0', '1.2', '1.4', '1.6', '1.8'])

    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound1_array=[]
    upper_bound2_array=[]
    observed_array=[]
    for k in range(5):
        upper_bound1_array.append(results[k][0])
        upper_bound2_array.append(results[k][1])
        observed_array.append(results[k][2])
    
    print(upper_bound1_array)
    print(upper_bound2_array)
    print(observed_array)
    # return
    ax.plot(x, upper_bound1_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound 1')
    # ax.scatter(x, upper_bound1_array, color="red", s=100)  # Add dots to represent data points

    # Plot the upper bound line
    ax.plot(x, upper_bound2_array, marker="v", color="#f6b57b", markersize=12, linewidth=2, label='Upper Bound 2')
    # ax.scatter(x, upper_bound2_array, color="red", s=100)  # Add dots to represent data points

    # Plot the observed line
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # Set the axis labels and title
    ax.set_xlabel(""+r'$T_i^W/T_i^B$'+"", fontsize=20)
    ax.set_ylabel("Passing Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # Add a grid to the chart
    ax.grid(axis="y", linestyle='--', alpha=0.7)

    # Customize the tick font sizes
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')    
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('./passing_latency_period_factor_1.svg', format='svg')

    # Show the plot
    # plt.show() 

def line_chart_plot_varied_period(results):
    ## Line Chart
    x = np.array(['[10,100]', '[20,100]', '[30,100]', '[40,100]', '[50,100]'])

    # Create a figure and axis object
    fig, ax = plt.subplots(figsize=(7, 4))

    # Plot the time disparity line
    upper_bound1_array=[]
    upper_bound2_array=[]
    observed_array=[]
    for k in range(5):
        upper_bound1_array.append(results[k][0])
        upper_bound2_array.append(results[k][1])
        observed_array.append(results[k][2])
    
    print(upper_bound1_array)
    print(upper_bound2_array)
    print(observed_array)
    # return
    ax.plot(x, upper_bound1_array, marker="o", color="#e54d4c", markersize=12, linewidth=2, label='Upper Bound 1')
    # ax.scatter(x, upper_bound1_array, color="red", s=100)  # Add dots to represent data points

    # Plot the upper bound line
    ax.plot(x, upper_bound2_array, marker="v", color="#f6b57b", markersize=12, linewidth=2, label='Upper Bound 2')
    # ax.scatter(x, upper_bound2_array, color="red", s=100)  # Add dots to represent data points

    # Plot the observed line
    ax.plot(x, observed_array, marker="d", color="#2b9fc9", markersize=12, linewidth=2, label='Observed')
    # ax.scatter(x, observed_array, color="red", s=100)  # Add dots to represent data points
    
    # Set the axis labels and title
    ax.set_xlabel('Range of Period (ms)', fontsize=20)
    ax.set_ylabel("Passing Latency (x"+r'$10^2$'+" ms)", fontsize=20)
    # ax.set_title('Delay Analysis', fontsize=16, fontweight='bold')

    # Add a grid to the chart
    ax.grid(axis="y", linestyle='--', alpha=0.7)

    # Customize the tick font sizes
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Set a custom y-axis range
    # ax.set_ylim(0, 25)

    # Add a legend and customize its font size
    ax.legend(fontsize=20, loc='lower right')    
    
    # Show only integer values on the y-axis
    # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    # ax.yaxis.set_major_formatter('{:.0f}'.format)
    ax.set_ylim(bottom=0)

    # Save the chart
    plt.savefig('./passing_latency_varied_period_1.svg', format='svg')

    # Show the plot
    # plt.show()    