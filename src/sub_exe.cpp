#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <string> 

#include "subscriber_latency.h"

using namespace synchronizer;
using namespace std;

int read_period(string file_name)
{
    std::ifstream file(file_name);
    int num;
    if (file.is_open()) {
        std::string line;
        std::getline(file, line);
        try {
            num = std::stoi(line); // Convert the line to an integer using std::stoi
        } catch (std::exception const& e) {
            std::cerr << "Error converting file contents to integer: " << e.what() << std::endl;
            exit(1);
            return -1;
        }
        file.close();
        return num;
    } else {
        std::cerr << "Unable to open file" << std::endl;
        exit(1);
        return -1;
    }
}

int main(int argc, char * argv[])
{
    ////////////////////////////////////////////////////////////////////////////////////////
    int channel_num = atoi(argv[1]);
    string result_path = argv[2];
    string periods_path = argv[3];
    
    ////////////////////////////////////////////////////////////////////////////////////////
    ofstream outfile_alg_;
    ofstream outfile_mdl_;

    outfile_mdl_.open(result_path + "/timestamp_mdl.txt", ios::app);
    if (!outfile_mdl_.is_open()) 
    {
        cout<<"Error opening file timestamp_mdl.txt! "<<endl;
    }

    // set output accuracy
    outfile_mdl_.setf(ios::fixed, ios::floatfield);
    outfile_mdl_.precision(9);

    int real_period1 = read_period(periods_path + "/period1.txt");
    int real_period2 = read_period(periods_path + "/period2.txt");

    switch(channel_num)
    {
    case 2:
    {
        rclcpp::init(argc, argv);
        rclcpp::executors::SingleThreadedExecutor executor;
        // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),50);

        auto sub_node = std::make_shared<SubscriberTopic2>(real_period1, real_period2, outfile_alg_, outfile_mdl_);
        executor.add_node(sub_node);

        executor.spin();

        executor.remove_node(sub_node);
        break;
    }
    case 3:
    {   
        rclcpp::init(argc, argv);
        rclcpp::executors::SingleThreadedExecutor executor;
        // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),50);

        int real_period3 = read_period(periods_path + "/period3.txt");
        auto sub_node = std::make_shared<SubscriberTopic3>(real_period1, real_period2, real_period3, outfile_alg_, outfile_mdl_);
        executor.add_node(sub_node);

        executor.spin();
        
        executor.remove_node(sub_node);

        break;
    }
    case 4:
    {
        rclcpp::init(argc, argv);
        rclcpp::executors::SingleThreadedExecutor executor;
        // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),50);

        int real_period3 = read_period(periods_path + "/period3.txt");
        int real_period4 = read_period(periods_path + "/period4.txt");
        auto sub_node = std::make_shared<SubscriberTopic4>(real_period1, real_period2, real_period3, real_period4, outfile_alg_, outfile_mdl_);
        executor.add_node(sub_node);

        executor.spin();

        executor.remove_node(sub_node);

        break;
    }
    case 5:
    {
        rclcpp::init(argc, argv);
        rclcpp::executors::SingleThreadedExecutor executor;
        // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),50);

        int real_period3 = read_period(periods_path + "/period3.txt");
        int real_period4 = read_period(periods_path + "/period4.txt");
        int real_period5 = read_period(periods_path + "/period5.txt");
        auto sub_node = std::make_shared<SubscriberTopic5>(real_period1, real_period2, real_period3, real_period4, real_period5, outfile_alg_, outfile_mdl_);
        executor.add_node(sub_node);

        executor.spin();

        executor.remove_node(sub_node);

        break;
    }
    case 6:
    {
        rclcpp::init(argc, argv);
        rclcpp::executors::SingleThreadedExecutor executor;
        // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),50);

        int real_period3 = read_period(periods_path + "/period3.txt");
        int real_period4 = read_period(periods_path + "/period4.txt");
        int real_period5 = read_period(periods_path + "/period5.txt");
        int real_period6 = read_period(periods_path + "/period6.txt");
        auto sub_node = std::make_shared<SubscriberTopic6>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, outfile_alg_, outfile_mdl_);
        executor.add_node(sub_node);

        executor.spin();

        executor.remove_node(sub_node);

        break;
    }
    case 7:
    {
        rclcpp::init(argc, argv);
        rclcpp::executors::SingleThreadedExecutor executor;
        // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),50);

        int real_period3 = read_period(periods_path + "/period3.txt");
        int real_period4 = read_period(periods_path + "/period4.txt");
        int real_period5 = read_period(periods_path + "/period5.txt");
        int real_period6 = read_period(periods_path + "/period6.txt");
        int real_period7 = read_period(periods_path + "/period7.txt");
        auto sub_node = std::make_shared<SubscriberTopic7>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, real_period7, outfile_alg_, outfile_mdl_);
        executor.add_node(sub_node);

        executor.spin();

        executor.remove_node(sub_node);

        break;
    }
    case 8:
    {
        rclcpp::init(argc, argv);
        rclcpp::executors::SingleThreadedExecutor executor;
        // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),50);

        int real_period3 = read_period(periods_path + "/period3.txt");
        int real_period4 = read_period(periods_path + "/period4.txt");
        int real_period5 = read_period(periods_path + "/period5.txt");
        int real_period6 = read_period(periods_path + "/period6.txt");
        int real_period7 = read_period(periods_path + "/period7.txt");
        int real_period8 = read_period(periods_path + "/period8.txt");
        auto sub_node = std::make_shared<SubscriberTopic8>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, real_period7, real_period8, outfile_alg_, outfile_mdl_);
        executor.add_node(sub_node);

        executor.spin();

        executor.remove_node(sub_node);

        break;
    }
    case 9:
    {    
        rclcpp::init(argc, argv);
        rclcpp::executors::SingleThreadedExecutor executor;
        // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),50);

        int real_period3 = read_period(periods_path + "/period3.txt");
        int real_period4 = read_period(periods_path + "/period4.txt");
        int real_period5 = read_period(periods_path + "/period5.txt");
        int real_period6 = read_period(periods_path + "/period6.txt");
        int real_period7 = read_period(periods_path + "/period7.txt");
        int real_period8 = read_period(periods_path + "/period8.txt");
        int real_period9 = read_period(periods_path + "/period9.txt");
        auto sub_node = std::make_shared<SubscriberTopic9>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, real_period7, real_period8, real_period9, outfile_alg_, outfile_mdl_);
        executor.add_node(sub_node);

        executor.spin();

        executor.remove_node(sub_node);
      
        break;
    }
    default:
        break;        
    }

    rclcpp::shutdown();
    return 0;
}
