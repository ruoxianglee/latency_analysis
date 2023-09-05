#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <unistd.h> // include the header file for sleep()

#include <string> 

#include "publisher.h"

using namespace synchronizer;
using namespace std;

int main(int argc, char * argv[])
{
    string topic_name;
    ofstream outfile_period;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    default_random_engine gen(seed);

    ////////////////////////////////////////////////////////////////////////////////////////
    int lower_limit = atoi(argv[1]);
    string periods_path = argv[2];

    outfile_period.open(periods_path + "/period7.txt", ios::out);
    if (outfile_period.is_open())
    {
        cout << "Open file for Period 7 successfully." << endl;
    }
    else {
        cerr << "Error: Unable to open file for writing." << endl;
        exit(1);  // exit with error status
    }

    ////////////////////////////////////////////////////////////////////////////////////////
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),50);

    uniform_int_distribution<unsigned> perd(lower_limit, 100);
    int real_period = perd(gen);
    outfile_period << real_period << endl;      // write the period to file
    outfile_period.close();     // close the file
    topic_name = "topic" + to_string(7);
    auto pub_node = std::make_shared<Publisher>(topic_name, real_period);
    executor.add_node(pub_node);

    executor.spin();

    executor.remove_node(pub_node);

    rclcpp::shutdown();
    
    return 0;
}
