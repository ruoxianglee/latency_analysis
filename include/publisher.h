/*
Delay operation executes here.
*/
#include <chrono>
#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include "helper.h"

#include <random>
using namespace std;

using namespace std::chrono_literals;

namespace synchronizer
{
class Publisher : public rclcpp::Node
{
public:
  Publisher(string topic_name, int real_period)
  : Node(topic_name.c_str()), count_(0), thread_created_(false), real_period_(real_period)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(topic_name.c_str(), 10);

    this->declare_parameter("msg_delay_opt", 0);
    this->declare_parameter("delay_upper", 0);
    // this->declare_parameter("sample_upper", 50);

    this->get_parameter("msg_delay_opt", msg_delay_opt_);
    this->get_parameter("delay_upper", delay_upper_);
    // this->get_parameter("sample_upper", sample_upper_);


    RCLCPP_INFO(this->get_logger(), "Delay option has been set as: %d, with delay upper limit: %d.", msg_delay_opt_, delay_upper_);

    this->declare_parameter("period_factor", 1.0);
    this->declare_parameter("msg_period_bias_flag", false);

    this->get_parameter("period_factor", period_factor_);
    this->get_parameter("msg_period_bias_flag", msg_period_bias_flag_);

    if(msg_period_bias_flag_)
    {
      RCLCPP_INFO(this->get_logger(), "Period bias has been set as: %.1f", period_factor_);
    }

    RCLCPP_INFO(this->get_logger(), "Period of %s: %d", topic_name.c_str(), real_period_);

    period_bias_upper_ = (period_factor_ - 1) * real_period_;
    period_ = 1ms * real_period_;

    Thread = new std::thread(&Publisher::timer_callback,this);
  }

private:
  void thread_create()
  {
    if(!thread_created_)
    {
      thread_created_ = true;
      Thread = new std::thread(&Publisher::timer_callback,this);
      // Thread->join();
    }
  }

  void timer_callback()
  { 
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    default_random_engine e(seed);
    uniform_int_distribution<unsigned> bias(0, period_bias_upper_); // bias for T
    uniform_int_distribution<unsigned> bias_flag(0, 1); // bias or not

    while(rclcpp::ok())
    {
      int period_bias = 0;
      if(msg_period_bias_flag_)
      {
        int flag = bias_flag(e);
        int b = bias(e);
        period_bias = flag * b;
      }
      
      rclcpp::sleep_for(std::chrono::milliseconds(real_period_ + period_bias - int(delay_previous_)));
      rclcpp::Time now = this->now();

      std::string symbol = std::to_string(count_);
      count_++;
      
      if(count_ > 100)
        publisher(symbol, now, real_period_ + period_bias);
    }
  }

  void publisher(std::string symbol,rclcpp::Time now, int period_now)
  {
    auto message=sensor_msgs::msg::JointState();
    message.name.push_back("pub1:"+symbol);
    message.header.stamp = now;
   
    message.timing.timestamp = now;
    // event_generator(message.timing.event_id, message.timing.earliest_sample);
    
    if(msg_delay_opt_ == 1)
    {
        delay_some_time(delay_previous_, period_previous_); // delay 1-40ms always
    }
    else if(msg_delay_opt_ == 2)
    {
        delay_random_time(delay_previous_, period_previous_, delay_upper_); // delay 0-upper ms
    }

    publisher_->publish(message);

    period_previous_ = period_now;
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

  size_t count_;
  bool thread_created_;

  int real_period_;
  std::chrono::milliseconds period_;

  double period_factor_;
  bool msg_period_bias_flag_;
  double period_bias_upper_;
  int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
  int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
  double delay_previous_ = 0.;
  int period_previous_ = 0;

  std::thread* Thread;
};

}