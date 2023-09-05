/*
Delay operation executes in publisher.
Add more timing tracing points.
*/

#include <string>
#include <fstream>
#include <vector>
#include <cfloat>

#include <message_filters/subscriber.h>  
#include <message_filters/synchronizer.h>  
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/approximate_time_model.h>

#include <sensor_msgs/msg/joint_state.hpp>

#include "signal.h"
#include "helper.h"

using namespace message_filters;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;
using std::placeholders::_6;
using std::placeholders::_7;
using std::placeholders::_8;
using std::placeholders::_9;

namespace synchronizer
{

#define buffer_size 10000000

class SubscriberTopic2
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic2(int period1, int period2, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic2"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl),
        alg_sync_(buffer_size), mdl_sync_(), period1_(period1), period2_(period2),
        count_alg_(0), count_mdl_(0), delay_previous1_(0), delay_previous2_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), previous_timestamp1_(0), previous_timestamp2_(0), observed_wcp1_(0), observed_wcp2_(0)
    {
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic2::callback1, this, _1));
        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic2::callback2, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic2::mdl_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d", period1, period2);

        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);

        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic2()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2)
    {
        rclcpp::Time output_time = this->now();

        if(count_mdl_ >= num_published_set_)
        {
            cout << "Approximate Time Model has got enough published sets !" << endl;

            outfile_mdl_ << observed_bcp1_ << " " << observed_bcp2_ << " ";
            outfile_mdl_ << observed_wcp1_ << " " << observed_wcp2_ << " ";
            outfile_mdl_ << min_trans_delay1_ /  Mstos << " " << min_trans_delay2_ /  Mstos << " ";
            outfile_mdl_ << max_trans_delay1_ /  Mstos << " " << max_trans_delay2_ /  Mstos << " ";
            outfile_mdl_ << max_passing_latency1_ << " " << max_passing_latency2_ << " ";
            outfile_mdl_ << max_reaction_latency1_ << " " << max_reaction_latency2_ << endl;

            kill(getpid(),SIGINT);
            return;
        }
        count_mdl_++;

        if (count_mdl_ > 1)
        {
            double reaction_latency1 = (output_time - previous_arrival_time1_).seconds();
            if (reaction_latency1 > max_reaction_latency1_)
            {
                max_reaction_latency1_ = reaction_latency1;
            }

            double reaction_latency2 = (output_time - previous_arrival_time2_).seconds();
            if (reaction_latency2 > max_reaction_latency2_)
            {
                max_reaction_latency2_ = reaction_latency2;
            }
        }

        previous_arrival_time1_ = msg1->timing.arrival_time;
        previous_arrival_time2_ = msg2->timing.arrival_time;

        // Passing latency logging
        rclcpp::Time start_time1 = msg1->timing.arrival_time;
        rclcpp::Time start_time2 = msg2->timing.arrival_time;

        double latency1 = (output_time - start_time1).seconds();
        double latency2 = (output_time - start_time2).seconds();

        if (latency1 > max_passing_latency1_)
        {
            max_passing_latency1_ = latency1;
        }

        if (latency2 > max_passing_latency2_)
        {
            max_passing_latency2_ = latency2;
        }
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous1_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<0>(msg);

        if (delay_previous1_ > max_trans_delay1_)
        {
            max_trans_delay1_ = delay_previous1_;
        }

        if (delay_previous1_ < min_trans_delay1_)
        {
            min_trans_delay1_ = delay_previous1_;
        }
        
        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }

                if(observed_now < observed_bcp1_)
                {
                    observed_bcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous2_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<1>(msg);

        if (delay_previous2_ > max_trans_delay2_)
        {
            max_trans_delay2_ = delay_previous2_;
        }

        if (delay_previous2_ < min_trans_delay2_)
        {
            min_trans_delay2_ = delay_previous2_;
        }

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }

                if(observed_now < observed_bcp2_)
                {
                    observed_bcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_;

    int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets

    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_;
    double observed_wcp1_, observed_wcp2_;

    double observed_bcp1_ = FLT_MAX, observed_bcp2_ = FLT_MAX;

    double max_trans_delay1_ = 0., max_trans_delay2_ = 0.;
    double min_trans_delay1_ = FLT_MAX, min_trans_delay2_ = FLT_MAX;
    double max_passing_latency1_ = 0., max_passing_latency2_ = 0.;
    
    double max_reaction_latency1_ = 0., max_reaction_latency2_ = 0.;
    rclcpp::Time previous_arrival_time1_, previous_arrival_time2_;
};

class SubscriberTopic3
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic3(int period1, int period2, int period3, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic3"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl),
        alg_sync_(buffer_size), mdl_sync_(), period1_(period1), period2_(period2), period3_(period3),
        count_alg_(0), count_mdl_(0), delay_previous1_(0.), delay_previous2_(0), delay_previous3_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), previous_timestamp1_(0), previous_timestamp2_(0), previous_timestamp3_(0), observed_wcp1_(0), observed_wcp2_(0), observed_wcp3_(0)
    {
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic3::callback1, this, _1));

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic3::callback2, this, _1));

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic3::callback3, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic3::mdl_callback, this, _1, _2, _3));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d", period1, period2, period3);

        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        mdl_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        
        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic3()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3)
    {
        rclcpp::Time output_time = this->now();

        if(count_mdl_ >= num_published_set_)
        {
            cout << "Approximate Time Model has got enough published sets !" << endl;

            outfile_mdl_ << observed_bcp1_ << " " << observed_bcp2_ << " " << observed_bcp3_ << " ";
            outfile_mdl_ << observed_wcp1_ << " " << observed_wcp2_ << " " << observed_wcp3_ << " ";
            outfile_mdl_ << min_trans_delay1_ /  Mstos << " " << min_trans_delay2_ /  Mstos << " " << min_trans_delay3_ /  Mstos << " ";
            outfile_mdl_ << max_trans_delay1_ /  Mstos << " " << max_trans_delay2_ /  Mstos << " " << max_trans_delay3_ /  Mstos << " ";
            outfile_mdl_ << max_passing_latency1_ << " " << max_passing_latency2_ << " " << max_passing_latency3_ << " ";
            outfile_mdl_ << max_reaction_latency1_ << " " << max_reaction_latency2_ << " " << max_reaction_latency3_ << endl;

            kill(getpid(),SIGINT);
            return;     
        }

        count_mdl_++;

        if (count_mdl_ > 1)
        {
            double reaction_latency1 = (output_time - previous_arrival_time1_).seconds();
            if (reaction_latency1 > max_reaction_latency1_)
            {
                max_reaction_latency1_ = reaction_latency1;
            }

            double reaction_latency2 = (output_time - previous_arrival_time2_).seconds();
            if (reaction_latency2 > max_reaction_latency2_)
            {
                max_reaction_latency2_ = reaction_latency2;
            }

            double reaction_latency3 = (output_time - previous_arrival_time3_).seconds();
            if (reaction_latency3 > max_reaction_latency3_)
            {
                max_reaction_latency3_ = reaction_latency3;
            }
        }

        previous_arrival_time1_ = msg1->timing.arrival_time;
        previous_arrival_time2_ = msg2->timing.arrival_time;
        previous_arrival_time3_ = msg3->timing.arrival_time;

        // Passing latency logging
        rclcpp::Time start_time1 = msg1->timing.arrival_time;
        rclcpp::Time start_time2 = msg2->timing.arrival_time;
        rclcpp::Time start_time3 = msg3->timing.arrival_time;

        double latency1 = (output_time - start_time1).seconds();
        double latency2 = (output_time - start_time2).seconds();
        double latency3 = (output_time - start_time3).seconds();

        if (latency1 > max_passing_latency1_)
        {
            max_passing_latency1_ = latency1;
        }

        if (latency2 > max_passing_latency2_)
        {
            max_passing_latency2_ = latency2;
        }

        if (latency3 > max_passing_latency3_)
        {
            max_passing_latency3_ = latency3;
        }
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {   
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous1_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<0>(msg);

        if (delay_previous1_ > max_trans_delay1_)
        {
            max_trans_delay1_ = delay_previous1_;
        }

        if (delay_previous1_ < min_trans_delay1_)
        {
            min_trans_delay1_ = delay_previous1_;
        }

        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }

                if(observed_now < observed_bcp1_)
                {
                    observed_bcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous2_ = absolute_delay.seconds() * Mstos;   

        mdl_sync_.add<1>(msg);

        if (delay_previous2_ > max_trans_delay2_)
        {
            max_trans_delay2_ = delay_previous2_;
        }

        if (delay_previous2_ < min_trans_delay2_)
        {
            min_trans_delay2_ = delay_previous2_;
        }

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }

                if(observed_now < observed_bcp2_)
                {
                    observed_bcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }
    }

    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous3_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<2>(msg);

        if (delay_previous3_ > max_trans_delay3_)
        {
            max_trans_delay3_ = delay_previous3_;
        }

        if (delay_previous3_ < min_trans_delay3_)
        {
            min_trans_delay3_ = delay_previous3_;
        }

        double topic3_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp3_ > 0)
            {
                double observed_now = topic3_timestamp - previous_timestamp3_;
                if(observed_now > observed_wcp3_)
                {
                    observed_wcp3_ = observed_now;
                }

                if(observed_now < observed_bcp3_)
                {
                    observed_bcp3_ = observed_now;
                }
            }
            previous_timestamp3_ = topic3_timestamp;
        }
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic3_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_, period3_;

    int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets    
    
    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_, delay_previous3_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_;

    double observed_bcp1_ = FLT_MAX, observed_bcp2_ = FLT_MAX, observed_bcp3_ = FLT_MAX;

    double max_trans_delay1_ = 0., max_trans_delay2_ = 0., max_trans_delay3_ = 0.;
    double min_trans_delay1_ = FLT_MAX, min_trans_delay2_ = FLT_MAX, min_trans_delay3_ = FLT_MAX;
    double max_passing_latency1_ = 0., max_passing_latency2_ = 0., max_passing_latency3_ = 0.;

    double max_reaction_latency1_ = 0., max_reaction_latency2_ = 0., max_reaction_latency3_ = 0.;
    rclcpp::Time previous_arrival_time1_, previous_arrival_time2_, previous_arrival_time3_;
};

class SubscriberTopic4
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic4(int period1, int period2, int period3, int period4, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic4"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl), 
        alg_sync_(buffer_size), mdl_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4),
        count_alg_(0), count_mdl_(0), 
        delay_previous1_(0), delay_previous2_(0), delay_previous3_(0), delay_previous4_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), 
        previous_timestamp1_(0), previous_timestamp2_(0), previous_timestamp3_(0), previous_timestamp4_(0), 
        observed_wcp1_(0), observed_wcp2_(0), observed_wcp3_(0), observed_wcp4_(0)
    {
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic4::callback1, this, _1));

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic4::callback2, this, _1));

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic4::callback3, this, _1));

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic4::callback4, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic4::mdl_callback, this, _1, _2, _3, _4));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d",
                    period1, period2, period3, period4);

        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        mdl_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        mdl_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);

        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic4()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4)
    {
        rclcpp::Time output_time = this->now();

        if(count_mdl_ >= num_published_set_)
        {
            cout << "Approximate Time Model has got enough published sets !" << endl;

            outfile_mdl_ << observed_bcp1_ << " " << observed_bcp2_ << " " << observed_bcp3_ << " " << observed_bcp4_ << " ";
            outfile_mdl_ << observed_wcp1_ << " " << observed_wcp2_ << " " << observed_wcp3_ << " " << observed_wcp4_ << " ";
            outfile_mdl_ << min_trans_delay1_ /  Mstos << " " << min_trans_delay2_ /  Mstos << " " << min_trans_delay3_ /  Mstos << " " << min_trans_delay4_ /  Mstos << " ";
            outfile_mdl_ << max_trans_delay1_ /  Mstos << " " << max_trans_delay2_ /  Mstos << " " << max_trans_delay3_ /  Mstos << " " << max_trans_delay4_ /  Mstos << " ";
            outfile_mdl_ << max_passing_latency1_ << " " << max_passing_latency2_ << " " << max_passing_latency3_ << " " << max_passing_latency4_ << " ";
            outfile_mdl_ << max_reaction_latency1_ << " " << max_reaction_latency2_ << " " << max_reaction_latency3_ << " " << max_reaction_latency4_ << endl;

            kill(getpid(),SIGINT);
            return;    
        }
        count_mdl_++;

        if (count_mdl_ > 1)
        {
            double reaction_latency1 = (output_time - previous_arrival_time1_).seconds();
            if (reaction_latency1 > max_reaction_latency1_)
            {
                max_reaction_latency1_ = reaction_latency1;
            }

            double reaction_latency2 = (output_time - previous_arrival_time2_).seconds();
            if (reaction_latency2 > max_reaction_latency2_)
            {
                max_reaction_latency2_ = reaction_latency2;
            }

            double reaction_latency3 = (output_time - previous_arrival_time3_).seconds();
            if (reaction_latency3 > max_reaction_latency3_)
            {
                max_reaction_latency3_ = reaction_latency3;
            }

            double reaction_latency4 = (output_time - previous_arrival_time4_).seconds();
            if (reaction_latency4 > max_reaction_latency4_)
            {
                max_reaction_latency4_ = reaction_latency4;
            }
        }

        previous_arrival_time1_ = msg1->timing.arrival_time;
        previous_arrival_time2_ = msg2->timing.arrival_time;
        previous_arrival_time3_ = msg3->timing.arrival_time;
        previous_arrival_time4_ = msg4->timing.arrival_time;

        // Passing latency logging
        rclcpp::Time start_time1 = msg1->timing.arrival_time;
        rclcpp::Time start_time2 = msg2->timing.arrival_time;
        rclcpp::Time start_time3 = msg3->timing.arrival_time;
        rclcpp::Time start_time4 = msg4->timing.arrival_time;

        double latency1 = (output_time - start_time1).seconds();
        double latency2 = (output_time - start_time2).seconds();
        double latency3 = (output_time - start_time3).seconds();
        double latency4 = (output_time - start_time4).seconds();

        if (latency1 > max_passing_latency1_)
        {
            max_passing_latency1_ = latency1;
        }

        if (latency2 > max_passing_latency2_)
        {
            max_passing_latency2_ = latency2;
        }

        if (latency3 > max_passing_latency3_)
        {
            max_passing_latency3_ = latency3;
        }

        if (latency4 > max_passing_latency4_)
        {
            max_passing_latency4_ = latency4;
        }
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {   
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous1_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<0>(msg);

        if (delay_previous1_ > max_trans_delay1_)
        {
            max_trans_delay1_ = delay_previous1_;
        }

        if (delay_previous1_ < min_trans_delay1_)
        {
            min_trans_delay1_ = delay_previous1_;
        }

        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }

                if(observed_now < observed_bcp1_)
                {
                    observed_bcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous2_ = absolute_delay.seconds() * Mstos;   

        mdl_sync_.add<1>(msg);

        if (delay_previous2_ > max_trans_delay2_)
        {
            max_trans_delay2_ = delay_previous2_;
        }

        if (delay_previous2_ < min_trans_delay2_)
        {
            min_trans_delay2_ = delay_previous2_;
        }

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }

                if(observed_now < observed_bcp2_)
                {
                    observed_bcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }
    }

    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous3_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<2>(msg);

        if (delay_previous3_ > max_trans_delay3_)
        {
            max_trans_delay3_ = delay_previous3_;
        }

        if (delay_previous3_ < min_trans_delay3_)
        {
            min_trans_delay3_ = delay_previous3_;
        }

        double topic3_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp3_ > 0)
            {
                double observed_now = topic3_timestamp - previous_timestamp3_;
                if(observed_now > observed_wcp3_)
                {
                    observed_wcp3_ = observed_now;
                }

                if(observed_now < observed_bcp3_)
                {
                    observed_bcp3_ = observed_now;
                }
            }
            previous_timestamp3_ = topic3_timestamp;
        }
    }

    void callback4(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous4_ = absolute_delay.seconds() * Mstos; 

        mdl_sync_.add<3>(msg);

        if (delay_previous4_ > max_trans_delay4_)
        {
            max_trans_delay4_ = delay_previous4_;
        }

        if (delay_previous4_ < min_trans_delay4_)
        {
            min_trans_delay4_ = delay_previous4_;
        }

        double topic4_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp4_ > 0)
            {
                double observed_now = topic4_timestamp - previous_timestamp4_;
                if(observed_now > observed_wcp4_)
                {
                    observed_wcp4_ = observed_now;
                }

                if(observed_now < observed_bcp4_)
                {
                    observed_bcp4_ = observed_now;
                }
            }
            previous_timestamp4_ = topic4_timestamp;
        }
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic3_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic4_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_, period3_, period4_;

    int msg_delay_opt_; // Message delay options. 0: no delay,2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets
    
    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_, delay_previous3_, delay_previous4_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_, previous_timestamp4_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_, observed_wcp4_;

    double observed_bcp1_ = FLT_MAX, observed_bcp2_ = FLT_MAX, observed_bcp3_ = FLT_MAX, observed_bcp4_ = FLT_MAX;

    double max_trans_delay1_ = 0., max_trans_delay2_ = 0., max_trans_delay3_ = 0., max_trans_delay4_ = 0.;
    double min_trans_delay1_ = FLT_MAX, min_trans_delay2_ = FLT_MAX, min_trans_delay3_ = FLT_MAX, min_trans_delay4_ = FLT_MAX;
    double max_passing_latency1_ = 0., max_passing_latency2_ = 0., max_passing_latency3_ = 0., max_passing_latency4_ = 0.;

    double max_reaction_latency1_ = 0., max_reaction_latency2_ = 0., max_reaction_latency3_ = 0., max_reaction_latency4_ = 0.;
    rclcpp::Time previous_arrival_time1_, previous_arrival_time2_, previous_arrival_time3_, previous_arrival_time4_;
};

class SubscriberTopic5
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic5(int period1, int period2, int period3, int period4, int period5, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic5"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl),
        alg_sync_(buffer_size), mdl_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5),
        count_alg_(0), count_mdl_(0), 
        delay_previous1_(0), delay_previous2_(0), delay_previous3_(0), delay_previous4_(0), delay_previous5_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), 
        previous_timestamp1_(0), previous_timestamp2_(0), previous_timestamp3_(0), previous_timestamp4_(0), previous_timestamp5_(0),
        observed_wcp1_(0), observed_wcp2_(0), observed_wcp3_(0), observed_wcp4_(0), observed_wcp5_(0)
    {
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;
        
        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic5::callback1, this, _1));

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic5::callback2, this, _1));

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic5::callback3, this, _1));

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic5::callback4, this, _1));

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic5::callback5, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic5::mdl_callback, this, _1, _2, _3, _4, _5));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d",
                    period1, period2, period3, period4, period5);

        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        mdl_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        mdl_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        mdl_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        
        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic5()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5)
    {
        rclcpp::Time output_time = this->now();

        if(count_mdl_ >= num_published_set_)
        {
            cout << "Approximate Time Model has got enough published sets !" << endl;

            outfile_mdl_ << observed_bcp1_ << " " << observed_bcp2_ << " " << observed_bcp3_ << " " << observed_bcp4_ << " " << observed_bcp5_ << " ";
            outfile_mdl_ << observed_wcp1_ << " " << observed_wcp2_ << " " << observed_wcp3_ << " " << observed_wcp4_ << " " << observed_wcp5_ << " ";
            outfile_mdl_ << min_trans_delay1_ /  Mstos << " " << min_trans_delay2_ /  Mstos << " " << min_trans_delay3_ /  Mstos << " " << min_trans_delay4_ /  Mstos << " " << min_trans_delay5_ /  Mstos << " ";
            outfile_mdl_ << max_trans_delay1_ /  Mstos << " " << max_trans_delay2_ /  Mstos << " " << max_trans_delay3_ /  Mstos << " " << max_trans_delay4_ /  Mstos << " " << max_trans_delay5_ /  Mstos << " ";
            outfile_mdl_ << max_passing_latency1_ << " " << max_passing_latency2_ << " " << max_passing_latency3_ << " " << max_passing_latency4_ << " " << max_passing_latency5_ << " ";
            outfile_mdl_ << max_reaction_latency1_ << " " << max_reaction_latency2_ << " " << max_reaction_latency3_ << " " << max_reaction_latency4_ << " " << max_reaction_latency5_ << endl;

            kill(getpid(),SIGINT);
            return;    
        }
        count_mdl_++;

        if (count_mdl_ > 1)
        {
            double reaction_latency1 = (output_time - previous_arrival_time1_).seconds();
            if (reaction_latency1 > max_reaction_latency1_)
            {
                max_reaction_latency1_ = reaction_latency1;
            }

            double reaction_latency2 = (output_time - previous_arrival_time2_).seconds();
            if (reaction_latency2 > max_reaction_latency2_)
            {
                max_reaction_latency2_ = reaction_latency2;
            }

            double reaction_latency3 = (output_time - previous_arrival_time3_).seconds();
            if (reaction_latency3 > max_reaction_latency3_)
            {
                max_reaction_latency3_ = reaction_latency3;
            }

            double reaction_latency4 = (output_time - previous_arrival_time4_).seconds();
            if (reaction_latency4 > max_reaction_latency4_)
            {
                max_reaction_latency4_ = reaction_latency4;
            }

            double reaction_latency5 = (output_time - previous_arrival_time5_).seconds();
            if (reaction_latency5 > max_reaction_latency5_)
            {
                max_reaction_latency5_ = reaction_latency5;
            }
        }

        previous_arrival_time1_ = msg1->timing.arrival_time;
        previous_arrival_time2_ = msg2->timing.arrival_time;
        previous_arrival_time3_ = msg3->timing.arrival_time;
        previous_arrival_time4_ = msg4->timing.arrival_time;
        previous_arrival_time5_ = msg5->timing.arrival_time;

        // Passing latency logging
        rclcpp::Time start_time1 = msg1->timing.arrival_time;
        rclcpp::Time start_time2 = msg2->timing.arrival_time;
        rclcpp::Time start_time3 = msg3->timing.arrival_time;
        rclcpp::Time start_time4 = msg4->timing.arrival_time;
        rclcpp::Time start_time5 = msg5->timing.arrival_time;

        double latency1 = (output_time - start_time1).seconds();
        double latency2 = (output_time - start_time2).seconds();
        double latency3 = (output_time - start_time3).seconds();
        double latency4 = (output_time - start_time4).seconds();
        double latency5 = (output_time - start_time5).seconds();

        if (latency1 > max_passing_latency1_)
        {
            max_passing_latency1_ = latency1;
        }

        if (latency2 > max_passing_latency2_)
        {
            max_passing_latency2_ = latency2;
        }

        if (latency3 > max_passing_latency3_)
        {
            max_passing_latency3_ = latency3;
        }

        if (latency4 > max_passing_latency4_)
        {
            max_passing_latency4_ = latency4;
        }

        if (latency5 > max_passing_latency5_)
        {
            max_passing_latency5_ = latency5;
        }
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous1_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<0>(msg);

        if (delay_previous1_ > max_trans_delay1_)
        {
            max_trans_delay1_ = delay_previous1_;
        }

        if (delay_previous1_ < min_trans_delay1_)
        {
            min_trans_delay1_ = delay_previous1_;
        }

        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }

                if(observed_now < observed_bcp1_)
                {
                    observed_bcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous2_ = absolute_delay.seconds() * Mstos;   

        mdl_sync_.add<1>(msg);

        if (delay_previous2_ > max_trans_delay2_)
        {
            max_trans_delay2_ = delay_previous2_;
        }

        if (delay_previous2_ < min_trans_delay2_)
        {
            min_trans_delay2_ = delay_previous2_;
        }

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }

                if(observed_now < observed_bcp2_)
                {
                    observed_bcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }
    }

    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous3_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<2>(msg);

        if (delay_previous3_ > max_trans_delay3_)
        {
            max_trans_delay3_ = delay_previous3_;
        }

        if (delay_previous3_ < min_trans_delay3_)
        {
            min_trans_delay3_ = delay_previous3_;
        }

        double topic3_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp3_ > 0)
            {
                double observed_now = topic3_timestamp - previous_timestamp3_;
                if(observed_now > observed_wcp3_)
                {
                    observed_wcp3_ = observed_now;
                }

                if(observed_now < observed_bcp3_)
                {
                    observed_bcp3_ = observed_now;
                }
            }
            previous_timestamp3_ = topic3_timestamp;
        }
    }

    void callback4(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous4_ = absolute_delay.seconds() * Mstos; 

        mdl_sync_.add<3>(msg);

        if (delay_previous4_ > max_trans_delay4_)
        {
            max_trans_delay4_ = delay_previous4_;
        }

        if (delay_previous4_ < min_trans_delay4_)
        {
            min_trans_delay4_ = delay_previous4_;
        }

        double topic4_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp4_ > 0)
            {
                double observed_now = topic4_timestamp - previous_timestamp4_;
                if(observed_now > observed_wcp4_)
                {
                    observed_wcp4_ = observed_now;
                }

                if(observed_now < observed_bcp4_)
                {
                    observed_bcp4_ = observed_now;
                }
            }
            previous_timestamp4_ = topic4_timestamp;
        }
    }

    void callback5(sensor_msgs::msg::JointState::SharedPtr msg)
    {       
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous5_ = absolute_delay.seconds() * Mstos;
            
        mdl_sync_.add<4>(msg);

        if (delay_previous5_ > max_trans_delay5_)
        {
            max_trans_delay5_ = delay_previous5_;
        }

        if (delay_previous5_ < min_trans_delay5_)
        {
            min_trans_delay5_ = delay_previous5_;
        }

        double topic5_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp5_ > 0)
            {
                double observed_now = topic5_timestamp - previous_timestamp5_;
                if(observed_now > observed_wcp5_)
                {
                    observed_wcp5_ = observed_now;
                }

                if(observed_now < observed_bcp5_)
                {
                    observed_bcp5_ = observed_now;
                }
            }
            previous_timestamp5_ = topic5_timestamp;
        }
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_, period3_, period4_, period5_;

    int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets

    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_, delay_previous3_, delay_previous4_, delay_previous5_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_, previous_timestamp4_, previous_timestamp5_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_, observed_wcp4_, observed_wcp5_;

    double observed_bcp1_ = FLT_MAX, observed_bcp2_ = FLT_MAX, observed_bcp3_ = FLT_MAX, observed_bcp4_ = FLT_MAX, observed_bcp5_ = FLT_MAX;

    double max_trans_delay1_ = 0., max_trans_delay2_ = 0., max_trans_delay3_ = 0., max_trans_delay4_ = 0., max_trans_delay5_ = 0.;
    double min_trans_delay1_ = FLT_MAX, min_trans_delay2_ = FLT_MAX, min_trans_delay3_ = FLT_MAX, min_trans_delay4_ = FLT_MAX, min_trans_delay5_ = FLT_MAX;
    double max_passing_latency1_ = 0., max_passing_latency2_ = 0., max_passing_latency3_ = 0., max_passing_latency4_ = 0., max_passing_latency5_ = 0.;

    double max_reaction_latency1_ = 0., max_reaction_latency2_ = 0., max_reaction_latency3_ = 0., max_reaction_latency4_ = 0., max_reaction_latency5_ = 0.;
    rclcpp::Time previous_arrival_time1_, previous_arrival_time2_, previous_arrival_time3_, previous_arrival_time4_, previous_arrival_time5_;
};

class SubscriberTopic6
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic6(int period1, int period2, int period3, int period4, int period5, int period6, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic6"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl),
        alg_sync_(buffer_size), mdl_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5), period6_(period6),
        count_alg_(0), count_mdl_(0), 
        delay_previous1_(0), delay_previous2_(0), delay_previous3_(0), delay_previous4_(0), delay_previous5_(0), delay_previous6_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), 
        previous_timestamp1_(0), previous_timestamp2_(0), previous_timestamp3_(0), previous_timestamp4_(0), previous_timestamp5_(0), previous_timestamp6_(0), 
        observed_wcp1_(0), observed_wcp2_(0), observed_wcp3_(0), observed_wcp4_(0), observed_wcp5_(0), observed_wcp6_(0)
    {
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic6::callback1, this, _1));

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic6::callback2, this, _1));

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic6::callback3, this, _1));

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic6::callback4, this, _1));

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic6::callback5, this, _1));

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic6::callback6, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic6::mdl_callback, this, _1, _2, _3, _4, _5, _6));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d, Period 6: %d",
                    period1, period2, period3, period4, period5, period6);

        
        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        mdl_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        mdl_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        mdl_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        mdl_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        
        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic6()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6)
    {
        rclcpp::Time output_time = this->now();

        if(count_mdl_ >= num_published_set_)
        {
            cout << "Approximate Time Model has got enough published sets !" << endl;

            outfile_mdl_ << observed_bcp1_ << " " << observed_bcp2_ << " " << observed_bcp3_ << " " << observed_bcp4_ << " " << observed_bcp5_ << " " << observed_bcp6_ << " ";
            outfile_mdl_ << observed_wcp1_ << " " << observed_wcp2_ << " " << observed_wcp3_ << " " << observed_wcp4_ << " " << observed_wcp5_ << " " << observed_wcp6_ << " ";
            outfile_mdl_ << min_trans_delay1_ /  Mstos << " " << min_trans_delay2_ /  Mstos << " " << min_trans_delay3_ /  Mstos << " " << min_trans_delay4_ /  Mstos << " " << min_trans_delay5_ /  Mstos << " " << min_trans_delay6_ /  Mstos << " ";
            outfile_mdl_ << max_trans_delay1_ /  Mstos << " " << max_trans_delay2_ /  Mstos << " " << max_trans_delay3_ /  Mstos << " " << max_trans_delay4_ /  Mstos << " " << max_trans_delay5_ /  Mstos << " " << max_trans_delay6_ /  Mstos << " ";
            outfile_mdl_ << max_passing_latency1_ << " " << max_passing_latency2_ << " " << max_passing_latency3_ << " " << max_passing_latency4_ << " " << max_passing_latency5_ << " " << max_passing_latency6_ << " ";
            outfile_mdl_ << max_reaction_latency1_ << " " << max_reaction_latency2_ << " " << max_reaction_latency3_ << " " << max_reaction_latency4_ << " " << max_reaction_latency5_ << " " << max_reaction_latency6_ << endl;

            kill(getpid(),SIGINT);
            return;    
        }
        count_mdl_++;

        if (count_mdl_ > 1)
        {
            double reaction_latency1 = (output_time - previous_arrival_time1_).seconds();
            if (reaction_latency1 > max_reaction_latency1_)
            {
                max_reaction_latency1_ = reaction_latency1;
            }

            double reaction_latency2 = (output_time - previous_arrival_time2_).seconds();
            if (reaction_latency2 > max_reaction_latency2_)
            {
                max_reaction_latency2_ = reaction_latency2;
            }

            double reaction_latency3 = (output_time - previous_arrival_time3_).seconds();
            if (reaction_latency3 > max_reaction_latency3_)
            {
                max_reaction_latency3_ = reaction_latency3;
            }

            double reaction_latency4 = (output_time - previous_arrival_time4_).seconds();
            if (reaction_latency4 > max_reaction_latency4_)
            {
                max_reaction_latency4_ = reaction_latency4;
            }

            double reaction_latency5 = (output_time - previous_arrival_time5_).seconds();
            if (reaction_latency5 > max_reaction_latency5_)
            {
                max_reaction_latency5_ = reaction_latency5;
            }

            double reaction_latency6 = (output_time - previous_arrival_time6_).seconds();
            if (reaction_latency6 > max_reaction_latency6_)
            {
                max_reaction_latency6_ = reaction_latency6;
            }
        }

        previous_arrival_time1_ = msg1->timing.arrival_time;
        previous_arrival_time2_ = msg2->timing.arrival_time;
        previous_arrival_time3_ = msg3->timing.arrival_time;
        previous_arrival_time4_ = msg4->timing.arrival_time;
        previous_arrival_time5_ = msg5->timing.arrival_time;
        previous_arrival_time6_ = msg6->timing.arrival_time;

        // Passing latency logging
        rclcpp::Time start_time1 = msg1->timing.arrival_time;
        rclcpp::Time start_time2 = msg2->timing.arrival_time;
        rclcpp::Time start_time3 = msg3->timing.arrival_time;
        rclcpp::Time start_time4 = msg4->timing.arrival_time;
        rclcpp::Time start_time5 = msg5->timing.arrival_time;
        rclcpp::Time start_time6 = msg6->timing.arrival_time;

        double latency1 = (output_time - start_time1).seconds();
        double latency2 = (output_time - start_time2).seconds();
        double latency3 = (output_time - start_time3).seconds();
        double latency4 = (output_time - start_time4).seconds();
        double latency5 = (output_time - start_time5).seconds();
        double latency6 = (output_time - start_time6).seconds();

        if (latency1 > max_passing_latency1_)
        {
            max_passing_latency1_ = latency1;
        }

        if (latency2 > max_passing_latency2_)
        {
            max_passing_latency2_ = latency2;
        }

        if (latency3 > max_passing_latency3_)
        {
            max_passing_latency3_ = latency3;
        }

        if (latency4 > max_passing_latency4_)
        {
            max_passing_latency4_ = latency4;
        }

        if (latency5 > max_passing_latency5_)
        {
            max_passing_latency5_ = latency5;
        }

        if (latency6 > max_passing_latency6_)
        {
            max_passing_latency6_ = latency6;
        }
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {       
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous1_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<0>(msg);

        if (delay_previous1_ > max_trans_delay1_)
        {
            max_trans_delay1_ = delay_previous1_;
        }

        if (delay_previous1_ < min_trans_delay1_)
        {
            min_trans_delay1_ = delay_previous1_;
        }

        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }

                if(observed_now < observed_bcp1_)
                {
                    observed_bcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous2_ = absolute_delay.seconds() * Mstos;   

        mdl_sync_.add<1>(msg);

        if (delay_previous2_ > max_trans_delay2_)
        {
            max_trans_delay2_ = delay_previous2_;
        }

        if (delay_previous2_ < min_trans_delay2_)
        {
            min_trans_delay2_ = delay_previous2_;
        }

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }

                if(observed_now < observed_bcp2_)
                {
                    observed_bcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }
    }

    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous3_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<2>(msg);

        if (delay_previous3_ > max_trans_delay3_)
        {
            max_trans_delay3_ = delay_previous3_;
        }

        if (delay_previous3_ < min_trans_delay3_)
        {
            min_trans_delay3_ = delay_previous3_;
        }

        double topic3_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp3_ > 0)
            {
                double observed_now = topic3_timestamp - previous_timestamp3_;
                if(observed_now > observed_wcp3_)
                {
                    observed_wcp3_ = observed_now;
                }

                if(observed_now < observed_bcp3_)
                {
                    observed_bcp3_ = observed_now;
                }
            }
            previous_timestamp3_ = topic3_timestamp;
        }
    }

    void callback4(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous4_ = absolute_delay.seconds() * Mstos; 

        mdl_sync_.add<3>(msg);

        if (delay_previous4_ > max_trans_delay4_)
        {
            max_trans_delay4_ = delay_previous4_;
        }

        if (delay_previous4_ < min_trans_delay4_)
        {
            min_trans_delay4_ = delay_previous4_;
        }

        double topic4_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp4_ > 0)
            {
                double observed_now = topic4_timestamp - previous_timestamp4_;
                if(observed_now > observed_wcp4_)
                {
                    observed_wcp4_ = observed_now;
                }

                if(observed_now < observed_bcp4_)
                {
                    observed_bcp4_ = observed_now;
                }
            }
            previous_timestamp4_ = topic4_timestamp;
        }
    }

    void callback5(sensor_msgs::msg::JointState::SharedPtr msg)
    {       
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous5_ = absolute_delay.seconds() * Mstos;
            
        mdl_sync_.add<4>(msg);

        if (delay_previous5_ > max_trans_delay5_)
        {
            max_trans_delay5_ = delay_previous5_;
        }

        if (delay_previous5_ < min_trans_delay5_)
        {
            min_trans_delay5_ = delay_previous5_;
        }

        double topic5_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp5_ > 0)
            {
                double observed_now = topic5_timestamp - previous_timestamp5_;
                if(observed_now > observed_wcp5_)
                {
                    observed_wcp5_ = observed_now;
                }

                if(observed_now < observed_bcp5_)
                {
                    observed_bcp5_ = observed_now;
                }
            }
            previous_timestamp5_ = topic5_timestamp;
        }
    }

    void callback6(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous6_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<5>(msg);

        if (delay_previous6_ > max_trans_delay6_)
        {
            max_trans_delay6_ = delay_previous6_;
        }

        if (delay_previous6_ < min_trans_delay6_)
        {
            min_trans_delay6_ = delay_previous6_;
        }

        double topic6_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp6_ > 0)
            {
                double observed_now = topic6_timestamp - previous_timestamp6_;
                if(observed_now > observed_wcp6_)
                {
                    observed_wcp6_ = observed_now;
                }

                if(observed_now < observed_bcp6_)
                {
                    observed_bcp6_ = observed_now;
                }
            }
            previous_timestamp6_ = topic6_timestamp;
        }
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_, topic6_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_, period3_, period4_, period5_, period6_;

    int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets
    
    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_, delay_previous3_, delay_previous4_, delay_previous5_, delay_previous6_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_, previous_timestamp4_, previous_timestamp5_, previous_timestamp6_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_, observed_wcp4_, observed_wcp5_, observed_wcp6_;

    double observed_bcp1_ = FLT_MAX, observed_bcp2_ = FLT_MAX, observed_bcp3_ = FLT_MAX, observed_bcp4_ = FLT_MAX, observed_bcp5_ = FLT_MAX, observed_bcp6_ = FLT_MAX;

    double max_trans_delay1_ = 0., max_trans_delay2_ = 0., max_trans_delay3_ = 0., max_trans_delay4_ = 0., max_trans_delay5_ = 0., max_trans_delay6_ = 0.;
    double min_trans_delay1_ = FLT_MAX, min_trans_delay2_ = FLT_MAX, min_trans_delay3_ = FLT_MAX, min_trans_delay4_ = FLT_MAX, min_trans_delay5_ = FLT_MAX, min_trans_delay6_ = FLT_MAX;
    double max_passing_latency1_ = 0., max_passing_latency2_ = 0., max_passing_latency3_ = 0., max_passing_latency4_ = 0., max_passing_latency5_ = 0., max_passing_latency6_ = 0.;

    double max_reaction_latency1_ = 0., max_reaction_latency2_ = 0., max_reaction_latency3_ = 0., max_reaction_latency4_ = 0., max_reaction_latency5_ = 0., max_reaction_latency6_ = 0.;
    rclcpp::Time previous_arrival_time1_, previous_arrival_time2_, previous_arrival_time3_, previous_arrival_time4_, previous_arrival_time5_, previous_arrival_time6_;
};

class SubscriberTopic7
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic7(int period1, int period2, int period3, int period4, int period5, int period6, int period7, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic7"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl),
        alg_sync_(buffer_size), mdl_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5), period6_(period6), period7_(period7),
        count_alg_(0), count_mdl_(0), 
        delay_previous1_(0), delay_previous2_(0), delay_previous3_(0), delay_previous4_(0), delay_previous5_(0), delay_previous6_(0), delay_previous7_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), 
        previous_timestamp1_(0), previous_timestamp2_(0), previous_timestamp3_(0), previous_timestamp4_(0), previous_timestamp5_(0), previous_timestamp6_(0), previous_timestamp7_(0),
        observed_wcp1_(0), observed_wcp2_(0), observed_wcp3_(0), observed_wcp4_(0), observed_wcp5_(0), observed_wcp6_(0), observed_wcp7_(0)
    {
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic7::callback1, this, _1));

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic7::callback2, this, _1));

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic7::callback3, this, _1));

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic7::callback4, this, _1));

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic7::callback5, this, _1));

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic7::callback6, this, _1));

        topic7_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic7", 300, 
                                                                        std::bind(&SubscriberTopic7::callback7, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic7::mdl_callback, this, _1, _2, _3, _4, _5, _6, _7));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d, Period 6: %d, Period 7: %d",
                    period1, period2, period3, period4, period5, period6, period7);

        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        mdl_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        mdl_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        mdl_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        mdl_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        mdl_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);
        
        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic7()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7)
    {
        rclcpp::Time output_time = this->now();

        if(count_mdl_ >= num_published_set_)
        {
            cout << "Approximate Time Model has got enough published sets !" << endl;

            outfile_mdl_ << observed_bcp1_ << " " << observed_bcp2_ << " " << observed_bcp3_ << " " << observed_bcp4_ << " " << observed_bcp5_ << " " << observed_bcp6_ << " " << observed_bcp7_ << " ";
            outfile_mdl_ << observed_wcp1_ << " " << observed_wcp2_ << " " << observed_wcp3_ << " " << observed_wcp4_ << " " << observed_wcp5_ << " " << observed_wcp6_ << " " << observed_wcp7_ << " ";
            outfile_mdl_ << min_trans_delay1_ /  Mstos << " " << min_trans_delay2_ /  Mstos << " " << min_trans_delay3_ /  Mstos << " " << min_trans_delay4_ /  Mstos << " " << min_trans_delay5_ /  Mstos << " " << min_trans_delay6_ /  Mstos << " " << min_trans_delay7_ /  Mstos << " ";
            outfile_mdl_ << max_trans_delay1_ /  Mstos << " " << max_trans_delay2_ /  Mstos << " " << max_trans_delay3_ /  Mstos << " " << max_trans_delay4_ /  Mstos << " " << max_trans_delay5_ /  Mstos << " " << max_trans_delay6_ /  Mstos << " " << max_trans_delay7_ /  Mstos << " ";
            outfile_mdl_ << max_passing_latency1_ << " " << max_passing_latency2_ << " " << max_passing_latency3_ << " " << max_passing_latency4_ << " " << max_passing_latency5_ << " " << max_passing_latency6_ << " " << max_passing_latency7_ << " ";
            outfile_mdl_ << max_reaction_latency1_ << " " << max_reaction_latency2_ << " " << max_reaction_latency3_ << " " << max_reaction_latency4_ << " " << max_reaction_latency5_ << " " << max_reaction_latency6_ << " " << max_reaction_latency7_ << endl;

            kill(getpid(),SIGINT);
            return;    
        }
        count_mdl_++;

        if (count_mdl_ > 1)
        {
            double reaction_latency1 = (output_time - previous_arrival_time1_).seconds();
            if (reaction_latency1 > max_reaction_latency1_)
            {
                max_reaction_latency1_ = reaction_latency1;
            }

            double reaction_latency2 = (output_time - previous_arrival_time2_).seconds();
            if (reaction_latency2 > max_reaction_latency2_)
            {
                max_reaction_latency2_ = reaction_latency2;
            }

            double reaction_latency3 = (output_time - previous_arrival_time3_).seconds();
            if (reaction_latency3 > max_reaction_latency3_)
            {
                max_reaction_latency3_ = reaction_latency3;
            }

            double reaction_latency4 = (output_time - previous_arrival_time4_).seconds();
            if (reaction_latency4 > max_reaction_latency4_)
            {
                max_reaction_latency4_ = reaction_latency4;
            }

            double reaction_latency5 = (output_time - previous_arrival_time5_).seconds();
            if (reaction_latency5 > max_reaction_latency5_)
            {
                max_reaction_latency5_ = reaction_latency5;
            }

            double reaction_latency6 = (output_time - previous_arrival_time6_).seconds();
            if (reaction_latency6 > max_reaction_latency6_)
            {
                max_reaction_latency6_ = reaction_latency6;
            }

            double reaction_latency7 = (output_time - previous_arrival_time7_).seconds();
            if (reaction_latency7 > max_reaction_latency7_)
            {
                max_reaction_latency7_ = reaction_latency7;
            }
        }

        previous_arrival_time1_ = msg1->timing.arrival_time;
        previous_arrival_time2_ = msg2->timing.arrival_time;
        previous_arrival_time3_ = msg3->timing.arrival_time;
        previous_arrival_time4_ = msg4->timing.arrival_time;
        previous_arrival_time5_ = msg5->timing.arrival_time;
        previous_arrival_time6_ = msg6->timing.arrival_time;
        previous_arrival_time7_ = msg7->timing.arrival_time;

        // Passing latency logging
        rclcpp::Time start_time1 = msg1->timing.arrival_time;
        rclcpp::Time start_time2 = msg2->timing.arrival_time;
        rclcpp::Time start_time3 = msg3->timing.arrival_time;
        rclcpp::Time start_time4 = msg4->timing.arrival_time;
        rclcpp::Time start_time5 = msg5->timing.arrival_time;
        rclcpp::Time start_time6 = msg6->timing.arrival_time;
        rclcpp::Time start_time7 = msg7->timing.arrival_time;

        double latency1 = (output_time - start_time1).seconds();
        double latency2 = (output_time - start_time2).seconds();
        double latency3 = (output_time - start_time3).seconds();
        double latency4 = (output_time - start_time4).seconds();
        double latency5 = (output_time - start_time5).seconds();
        double latency6 = (output_time - start_time6).seconds();
        double latency7 = (output_time - start_time7).seconds();

        if (latency1 > max_passing_latency1_)
        {
            max_passing_latency1_ = latency1;
        }

        if (latency2 > max_passing_latency2_)
        {
            max_passing_latency2_ = latency2;
        }

        if (latency3 > max_passing_latency3_)
        {
            max_passing_latency3_ = latency3;
        }

        if (latency4 > max_passing_latency4_)
        {
            max_passing_latency4_ = latency4;
        }

        if (latency5 > max_passing_latency5_)
        {
            max_passing_latency5_ = latency5;
        }

        if (latency6 > max_passing_latency6_)
        {
            max_passing_latency6_ = latency6;
        }

        if (latency7 > max_passing_latency7_)
        {
            max_passing_latency7_ = latency7;
        }
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {       
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous1_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<0>(msg);

        if (delay_previous1_ > max_trans_delay1_)
        {
            max_trans_delay1_ = delay_previous1_;
        }

        if (delay_previous1_ < min_trans_delay1_)
        {
            min_trans_delay1_ = delay_previous1_;
        }

        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }

                if(observed_now < observed_bcp1_)
                {
                    observed_bcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous2_ = absolute_delay.seconds() * Mstos;   

        mdl_sync_.add<1>(msg);

        if (delay_previous2_ > max_trans_delay2_)
        {
            max_trans_delay2_ = delay_previous2_;
        }

        if (delay_previous2_ < min_trans_delay2_)
        {
            min_trans_delay2_ = delay_previous2_;
        }

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }

                if(observed_now < observed_bcp2_)
                {
                    observed_bcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }
    }

    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous3_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<2>(msg);

        if (delay_previous3_ > max_trans_delay3_)
        {
            max_trans_delay3_ = delay_previous3_;
        }

        if (delay_previous3_ < min_trans_delay3_)
        {
            min_trans_delay3_ = delay_previous3_;
        }

        double topic3_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp3_ > 0)
            {
                double observed_now = topic3_timestamp - previous_timestamp3_;
                if(observed_now > observed_wcp3_)
                {
                    observed_wcp3_ = observed_now;
                }

                if(observed_now < observed_bcp3_)
                {
                    observed_bcp3_ = observed_now;
                }
            }
            previous_timestamp3_ = topic3_timestamp;
        }
    }

    void callback4(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous4_ = absolute_delay.seconds() * Mstos; 

        mdl_sync_.add<3>(msg);

        if (delay_previous4_ > max_trans_delay4_)
        {
            max_trans_delay4_ = delay_previous4_;
        }

        if (delay_previous4_ < min_trans_delay4_)
        {
            min_trans_delay4_ = delay_previous4_;
        }

        double topic4_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp4_ > 0)
            {
                double observed_now = topic4_timestamp - previous_timestamp4_;
                if(observed_now > observed_wcp4_)
                {
                    observed_wcp4_ = observed_now;
                }

                if(observed_now < observed_bcp4_)
                {
                    observed_bcp4_ = observed_now;
                }
            }
            previous_timestamp4_ = topic4_timestamp;
        }
    }

    void callback5(sensor_msgs::msg::JointState::SharedPtr msg)
    {       
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous5_ = absolute_delay.seconds() * Mstos;
            
        mdl_sync_.add<4>(msg);

        if (delay_previous5_ > max_trans_delay5_)
        {
            max_trans_delay5_ = delay_previous5_;
        }

        if (delay_previous5_ < min_trans_delay5_)
        {
            min_trans_delay5_ = delay_previous5_;
        }

        double topic5_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp5_ > 0)
            {
                double observed_now = topic5_timestamp - previous_timestamp5_;
                if(observed_now > observed_wcp5_)
                {
                    observed_wcp5_ = observed_now;
                }

                if(observed_now < observed_bcp5_)
                {
                    observed_bcp5_ = observed_now;
                }
            }
            previous_timestamp5_ = topic5_timestamp;
        }
    }

    void callback6(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous6_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<5>(msg);

        if (delay_previous6_ > max_trans_delay6_)
        {
            max_trans_delay6_ = delay_previous6_;
        }

        if (delay_previous6_ < min_trans_delay6_)
        {
            min_trans_delay6_ = delay_previous6_;
        }

        double topic6_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp6_ > 0)
            {
                double observed_now = topic6_timestamp - previous_timestamp6_;
                if(observed_now > observed_wcp6_)
                {
                    observed_wcp6_ = observed_now;
                }

                if(observed_now < observed_bcp6_)
                {
                    observed_bcp6_ = observed_now;
                }
            }
            previous_timestamp6_ = topic6_timestamp;
        }
    }

    void callback7(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous7_ = absolute_delay.seconds() * Mstos;  

        mdl_sync_.add<6>(msg);

        if (delay_previous7_ > max_trans_delay7_)
        {
            max_trans_delay7_ = delay_previous7_;
        }

        if (delay_previous7_ < min_trans_delay7_)
        {
            min_trans_delay7_ = delay_previous7_;
        }

        double topic7_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp7_ > 0)
            {
                double observed_now = topic7_timestamp - previous_timestamp7_;
                if(observed_now > observed_wcp7_)
                {
                    observed_wcp7_ = observed_now;
                }

                if(observed_now < observed_bcp7_)
                {
                    observed_bcp7_ = observed_now;
                }
            }
            previous_timestamp7_ = topic7_timestamp;
        }
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_, topic6_sub_, topic7_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_, period3_, period4_, period5_, period6_, period7_;

    int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets
    
    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_, delay_previous3_, delay_previous4_, delay_previous5_, delay_previous6_, delay_previous7_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_, previous_timestamp4_, previous_timestamp5_, previous_timestamp6_, previous_timestamp7_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_, observed_wcp4_, observed_wcp5_, observed_wcp6_, observed_wcp7_;

    double observed_bcp1_ = FLT_MAX, observed_bcp2_ = FLT_MAX, observed_bcp3_ = FLT_MAX, observed_bcp4_ = FLT_MAX, observed_bcp5_ = FLT_MAX, observed_bcp6_ = FLT_MAX, observed_bcp7_ = FLT_MAX;

    double max_trans_delay1_ = 0., max_trans_delay2_ = 0., max_trans_delay3_ = 0., max_trans_delay4_ = 0., max_trans_delay5_ = 0., max_trans_delay6_ = 0., max_trans_delay7_ = 0.;
    double min_trans_delay1_ = FLT_MAX, min_trans_delay2_ = FLT_MAX, min_trans_delay3_ = FLT_MAX, min_trans_delay4_ = FLT_MAX, min_trans_delay5_ = FLT_MAX, min_trans_delay6_ = FLT_MAX, min_trans_delay7_ = FLT_MAX;
    double max_passing_latency1_ = 0., max_passing_latency2_ = 0., max_passing_latency3_ = 0., max_passing_latency4_ = 0., max_passing_latency5_ = 0., max_passing_latency6_ = 0., max_passing_latency7_ = 0.;

    double max_reaction_latency1_ = 0., max_reaction_latency2_ = 0., max_reaction_latency3_ = 0., max_reaction_latency4_ = 0., max_reaction_latency5_ = 0., max_reaction_latency6_ = 0., max_reaction_latency7_ = 0.;
    rclcpp::Time previous_arrival_time1_, previous_arrival_time2_, previous_arrival_time3_, previous_arrival_time4_, previous_arrival_time5_, previous_arrival_time6_, previous_arrival_time7_;
};

class SubscriberTopic8
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic8(int period1, int period2, int period3, int period4, int period5, int period6, int period7, int period8, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic8"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl),
        alg_sync_(buffer_size), mdl_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5), period6_(period6), period7_(period7), period8_(period8),
        count_alg_(0), count_mdl_(0), 
        delay_previous1_(0), delay_previous2_(0), delay_previous3_(0), delay_previous4_(0), delay_previous5_(0), delay_previous6_(0), delay_previous7_(0), delay_previous8_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), 
        previous_timestamp1_(0), previous_timestamp2_(0), previous_timestamp3_(0), previous_timestamp4_(0), previous_timestamp5_(0), previous_timestamp6_(0), previous_timestamp7_(0), previous_timestamp8_(0),
        observed_wcp1_(0), observed_wcp2_(0), observed_wcp3_(0), observed_wcp4_(0), observed_wcp5_(0), observed_wcp6_(0), observed_wcp7_(0), observed_wcp8_(0)
    {
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic8::callback1, this, _1));

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic8::callback2, this, _1));

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic8::callback3, this, _1));

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic8::callback4, this, _1));

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic8::callback5, this, _1));

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic8::callback6, this, _1));

        topic7_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic7", 300, 
                                                                        std::bind(&SubscriberTopic8::callback7, this, _1));

        topic8_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic8", 300, 
                                                                        std::bind(&SubscriberTopic8::callback8, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic8::mdl_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d, Period 6: %d, Period 7: %d, Period 8: %d",
                    period1, period2, period3, period4, period5, period6, period7, period8);

        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        mdl_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        mdl_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        mdl_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        mdl_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        mdl_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);
        mdl_sync_.setInterMessageLowerBound(7, PeriodBase * 0.001 * period8);
        
        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic8()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7, const sensor_msgs::msg::JointState::ConstSharedPtr& msg8)
    {
        rclcpp::Time output_time = this->now();

        if(count_mdl_ >= num_published_set_)
        {
            cout << "Approximate Time Model has got enough published sets !" << endl;

            outfile_mdl_ << observed_bcp1_ << " " << observed_bcp2_ << " " << observed_bcp3_ << " " << observed_bcp4_ << " " << observed_bcp5_ << " " << observed_bcp6_ << " " << observed_bcp7_ << " " << observed_bcp8_ << " ";
            outfile_mdl_ << observed_wcp1_ << " " << observed_wcp2_ << " " << observed_wcp3_ << " " << observed_wcp4_ << " " << observed_wcp5_ << " " << observed_wcp6_ << " " << observed_wcp7_ << " " << observed_wcp8_ << " ";
            outfile_mdl_ << min_trans_delay1_ /  Mstos << " " << min_trans_delay2_ /  Mstos << " " << min_trans_delay3_ /  Mstos << " " << min_trans_delay4_ /  Mstos << " " << min_trans_delay5_ /  Mstos << " " << min_trans_delay6_ /  Mstos << " " << min_trans_delay7_ /  Mstos << " " << min_trans_delay8_ /  Mstos << " ";
            outfile_mdl_ << max_trans_delay1_ /  Mstos << " " << max_trans_delay2_ /  Mstos << " " << max_trans_delay3_ /  Mstos << " " << max_trans_delay4_ /  Mstos << " " << max_trans_delay5_ /  Mstos << " " << max_trans_delay6_ /  Mstos << " " << max_trans_delay7_ /  Mstos << " " << max_trans_delay8_ /  Mstos << " ";
            outfile_mdl_ << max_passing_latency1_ << " " << max_passing_latency2_ << " " << max_passing_latency3_ << " " << max_passing_latency4_ << " " << max_passing_latency5_ << " " << max_passing_latency6_ << " " << max_passing_latency7_ << " " << max_passing_latency8_ << " ";
            outfile_mdl_ << max_reaction_latency1_ << " " << max_reaction_latency2_ << " " << max_reaction_latency3_ << " " << max_reaction_latency4_ << " " << max_reaction_latency5_ << " " << max_reaction_latency6_ << " " << max_reaction_latency7_ << " " << max_reaction_latency8_ << endl;

            kill(getpid(),SIGINT);
            return;    
        }
        count_mdl_++;

        if (count_mdl_ > 1)
        {
            double reaction_latency1 = (output_time - previous_arrival_time1_).seconds();
            if (reaction_latency1 > max_reaction_latency1_)
            {
                max_reaction_latency1_ = reaction_latency1;
            }

            double reaction_latency2 = (output_time - previous_arrival_time2_).seconds();
            if (reaction_latency2 > max_reaction_latency2_)
            {
                max_reaction_latency2_ = reaction_latency2;
            }

            double reaction_latency3 = (output_time - previous_arrival_time3_).seconds();
            if (reaction_latency3 > max_reaction_latency3_)
            {
                max_reaction_latency3_ = reaction_latency3;
            }

            double reaction_latency4 = (output_time - previous_arrival_time4_).seconds();
            if (reaction_latency4 > max_reaction_latency4_)
            {
                max_reaction_latency4_ = reaction_latency4;
            }

            double reaction_latency5 = (output_time - previous_arrival_time5_).seconds();
            if (reaction_latency5 > max_reaction_latency5_)
            {
                max_reaction_latency5_ = reaction_latency5;
            }

            double reaction_latency6 = (output_time - previous_arrival_time6_).seconds();
            if (reaction_latency6 > max_reaction_latency6_)
            {
                max_reaction_latency6_ = reaction_latency6;
            }

            double reaction_latency7 = (output_time - previous_arrival_time7_).seconds();
            if (reaction_latency7 > max_reaction_latency7_)
            {
                max_reaction_latency7_ = reaction_latency7;
            }

            double reaction_latency8 = (output_time - previous_arrival_time8_).seconds();
            if (reaction_latency8 > max_reaction_latency8_)
            {
                max_reaction_latency8_ = reaction_latency8;
            }
        }

        previous_arrival_time1_ = msg1->timing.arrival_time;
        previous_arrival_time2_ = msg2->timing.arrival_time;
        previous_arrival_time3_ = msg3->timing.arrival_time;
        previous_arrival_time4_ = msg4->timing.arrival_time;
        previous_arrival_time5_ = msg5->timing.arrival_time;
        previous_arrival_time6_ = msg6->timing.arrival_time;
        previous_arrival_time7_ = msg7->timing.arrival_time;
        previous_arrival_time8_ = msg8->timing.arrival_time;

        // Passing latency logging
        rclcpp::Time start_time1 = msg1->timing.arrival_time;
        rclcpp::Time start_time2 = msg2->timing.arrival_time;
        rclcpp::Time start_time3 = msg3->timing.arrival_time;
        rclcpp::Time start_time4 = msg4->timing.arrival_time;
        rclcpp::Time start_time5 = msg5->timing.arrival_time;
        rclcpp::Time start_time6 = msg6->timing.arrival_time;
        rclcpp::Time start_time7 = msg7->timing.arrival_time;
        rclcpp::Time start_time8 = msg8->timing.arrival_time;

        double latency1 = (output_time - start_time1).seconds();
        double latency2 = (output_time - start_time2).seconds();
        double latency3 = (output_time - start_time3).seconds();
        double latency4 = (output_time - start_time4).seconds();
        double latency5 = (output_time - start_time5).seconds();
        double latency6 = (output_time - start_time6).seconds();
        double latency7 = (output_time - start_time7).seconds();
        double latency8 = (output_time - start_time8).seconds();

        if (latency1 > max_passing_latency1_)
        {
            max_passing_latency1_ = latency1;
        }

        if (latency2 > max_passing_latency2_)
        {
            max_passing_latency2_ = latency2;
        }

        if (latency3 > max_passing_latency3_)
        {
            max_passing_latency3_ = latency3;
        }

        if (latency4 > max_passing_latency4_)
        {
            max_passing_latency4_ = latency4;
        }

        if (latency5 > max_passing_latency5_)
        {
            max_passing_latency5_ = latency5;
        }

        if (latency6 > max_passing_latency6_)
        {
            max_passing_latency6_ = latency6;
        }

        if (latency7 > max_passing_latency7_)
        {
            max_passing_latency7_ = latency7;
        }

        if (latency8 > max_passing_latency8_)
        {
            max_passing_latency8_ = latency8;
        }
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {       
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous1_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<0>(msg);

        if (delay_previous1_ > max_trans_delay1_)
        {
            max_trans_delay1_ = delay_previous1_;
        }

        if (delay_previous1_ < min_trans_delay1_)
        {
            min_trans_delay1_ = delay_previous1_;
        }

        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }

                if(observed_now < observed_bcp1_)
                {
                    observed_bcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous2_ = absolute_delay.seconds() * Mstos;   

        mdl_sync_.add<1>(msg);

        if (delay_previous2_ > max_trans_delay2_)
        {
            max_trans_delay2_ = delay_previous2_;
        }

        if (delay_previous2_ < min_trans_delay2_)
        {
            min_trans_delay2_ = delay_previous2_;
        }

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }

                if(observed_now < observed_bcp2_)
                {
                    observed_bcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }
    }

    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous3_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<2>(msg);

        if (delay_previous3_ > max_trans_delay3_)
        {
            max_trans_delay3_ = delay_previous3_;
        }

        if (delay_previous3_ < min_trans_delay3_)
        {
            min_trans_delay3_ = delay_previous3_;
        }

        double topic3_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp3_ > 0)
            {
                double observed_now = topic3_timestamp - previous_timestamp3_;
                if(observed_now > observed_wcp3_)
                {
                    observed_wcp3_ = observed_now;
                }

                if(observed_now < observed_bcp3_)
                {
                    observed_bcp3_ = observed_now;
                }
            }
            previous_timestamp3_ = topic3_timestamp;
        }  
    }

    void callback4(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous4_ = absolute_delay.seconds() * Mstos; 

        mdl_sync_.add<3>(msg);

        if (delay_previous4_ > max_trans_delay4_)
        {
            max_trans_delay4_ = delay_previous4_;
        }

        if (delay_previous4_ < min_trans_delay4_)
        {
            min_trans_delay4_ = delay_previous4_;
        }

        double topic4_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp4_ > 0)
            {
                double observed_now = topic4_timestamp - previous_timestamp4_;
                if(observed_now > observed_wcp4_)
                {
                    observed_wcp4_ = observed_now;
                }

                if(observed_now < observed_bcp4_)
                {
                    observed_bcp4_ = observed_now;
                }
            }
            previous_timestamp4_ = topic4_timestamp;
        }
    }

    void callback5(sensor_msgs::msg::JointState::SharedPtr msg)
    {    
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous5_ = absolute_delay.seconds() * Mstos;
            
        mdl_sync_.add<4>(msg);

        if (delay_previous5_ > max_trans_delay5_)
        {
            max_trans_delay5_ = delay_previous5_;
        }

        if (delay_previous5_ < min_trans_delay5_)
        {
            min_trans_delay5_ = delay_previous5_;
        }

        double topic5_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp5_ > 0)
            {
                double observed_now = topic5_timestamp - previous_timestamp5_;
                if(observed_now > observed_wcp5_)
                {
                    observed_wcp5_ = observed_now;
                }

                if(observed_now < observed_bcp5_)
                {
                    observed_bcp5_ = observed_now;
                }
            }
            previous_timestamp5_ = topic5_timestamp;
        }
    }

    void callback6(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous6_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<5>(msg);

        if (delay_previous6_ > max_trans_delay6_)
        {
            max_trans_delay6_ = delay_previous6_;
        }

        if (delay_previous6_ < min_trans_delay6_)
        {
            min_trans_delay6_ = delay_previous6_;
        }

        double topic6_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp6_ > 0)
            {
                double observed_now = topic6_timestamp - previous_timestamp6_;
                if(observed_now > observed_wcp6_)
                {
                    observed_wcp6_ = observed_now;
                }

                if(observed_now < observed_bcp6_)
                {
                    observed_bcp6_ = observed_now;
                }
            }
            previous_timestamp6_ = topic6_timestamp;
        }  
    }

    void callback7(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous7_ = absolute_delay.seconds() * Mstos;  

        mdl_sync_.add<6>(msg);

        if (delay_previous7_ > max_trans_delay7_)
        {
            max_trans_delay7_ = delay_previous7_;
        }

        if (delay_previous7_ < min_trans_delay7_)
        {
            min_trans_delay7_ = delay_previous7_;
        }

        double topic7_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp7_ > 0)
            {
                double observed_now = topic7_timestamp - previous_timestamp7_;
                if(observed_now > observed_wcp7_)
                {
                    observed_wcp7_ = observed_now;
                }

                if(observed_now < observed_bcp7_)
                {
                    observed_bcp7_ = observed_now;
                }
            }
            previous_timestamp7_ = topic7_timestamp;
        }
    }

    void callback8(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous8_ = absolute_delay.seconds() * Mstos;      

        mdl_sync_.add<7>(msg);

        if (delay_previous8_ > max_trans_delay8_)
        {
            max_trans_delay8_ = delay_previous8_;
        }

        if (delay_previous8_ < min_trans_delay8_)
        {
            min_trans_delay8_ = delay_previous8_;
        }

        double topic8_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp8_ > 0)
            {
                double observed_now = topic8_timestamp - previous_timestamp8_;
                if(observed_now > observed_wcp8_)
                {
                    observed_wcp8_ = observed_now;
                }

                if(observed_now < observed_bcp8_)
                {
                    observed_bcp8_ = observed_now;
                }
            }
            previous_timestamp8_ = topic8_timestamp;
        }
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_, topic6_sub_, topic7_sub_, topic8_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_, period3_, period4_, period5_, period6_, period7_, period8_;

    int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets
    
    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_, delay_previous3_, delay_previous4_, delay_previous5_, delay_previous6_, delay_previous7_, delay_previous8_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_, previous_timestamp4_, previous_timestamp5_, previous_timestamp6_, previous_timestamp7_, previous_timestamp8_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_, observed_wcp4_, observed_wcp5_, observed_wcp6_, observed_wcp7_, observed_wcp8_;

    double observed_bcp1_ = FLT_MAX, observed_bcp2_ = FLT_MAX, observed_bcp3_ = FLT_MAX, observed_bcp4_ = FLT_MAX, observed_bcp5_ = FLT_MAX, observed_bcp6_ = FLT_MAX, observed_bcp7_ = FLT_MAX, observed_bcp8_ = FLT_MAX;

    double max_trans_delay1_ = 0., max_trans_delay2_ = 0., max_trans_delay3_ = 0., max_trans_delay4_ = 0., max_trans_delay5_ = 0., max_trans_delay6_ = 0., max_trans_delay7_ = 0., max_trans_delay8_ = 0.;
    double min_trans_delay1_ = FLT_MAX, min_trans_delay2_ = FLT_MAX, min_trans_delay3_ = FLT_MAX, min_trans_delay4_ = FLT_MAX, min_trans_delay5_ = FLT_MAX, min_trans_delay6_ = FLT_MAX, min_trans_delay7_ = FLT_MAX, min_trans_delay8_ = FLT_MAX;
    double max_passing_latency1_ = 0., max_passing_latency2_ = 0., max_passing_latency3_ = 0., max_passing_latency4_ = 0., max_passing_latency5_ = 0., max_passing_latency6_ = 0., max_passing_latency7_ = 0., max_passing_latency8_ = 0.;

    double max_reaction_latency1_ = 0., max_reaction_latency2_ = 0., max_reaction_latency3_ = 0., max_reaction_latency4_ = 0., max_reaction_latency5_ = 0., max_reaction_latency6_ = 0., max_reaction_latency7_ = 0., max_reaction_latency8_ = 0.;
    rclcpp::Time previous_arrival_time1_, previous_arrival_time2_, previous_arrival_time3_, previous_arrival_time4_, previous_arrival_time5_, previous_arrival_time6_, previous_arrival_time7_, previous_arrival_time8_;
};

class SubscriberTopic9
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic9(int period1, int period2, int period3, int period4, int period5, int period6, int period7, int period8, int period9, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic9"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl), 
        alg_sync_(buffer_size), mdl_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5), period6_(period6), period7_(period7), period8_(period8), period9_(period9),
        count_alg_(0), count_mdl_(0), 
        delay_previous1_(0), delay_previous2_(0), delay_previous3_(0), delay_previous4_(0), delay_previous5_(0), delay_previous6_(0), delay_previous7_(0), delay_previous8_(0), delay_previous9_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), 
        previous_timestamp1_(0), previous_timestamp2_(0), previous_timestamp3_(0), previous_timestamp4_(0), previous_timestamp5_(0), previous_timestamp6_(0), previous_timestamp7_(0), previous_timestamp8_(0), previous_timestamp9_(0), 
        observed_wcp1_(0), observed_wcp2_(0), observed_wcp3_(0), observed_wcp4_(0), observed_wcp5_(0), observed_wcp6_(0), observed_wcp7_(0), observed_wcp8_(0), observed_wcp9_(0)
    {
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "required number of published sets: %d.", num_published_set_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic9::callback1, this, _1));

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic9::callback2, this, _1));

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic9::callback3, this, _1));

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic9::callback4, this, _1));

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic9::callback5, this, _1));

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic9::callback6, this, _1));

        topic7_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic7", 300, 
                                                                        std::bind(&SubscriberTopic9::callback7, this, _1));

        topic8_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic8", 300, 
                                                                        std::bind(&SubscriberTopic9::callback8, this, _1));

        topic9_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic8", 300, 
                                                                        std::bind(&SubscriberTopic9::callback9, this, _1));

        mdl_sync_.registerCallback(std::bind(&SubscriberTopic9::mdl_callback, this, _1, _2, _3, _4, _5, _6, _7, _8, _9));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d, Period 6: %d, Period 7: %d, Period 8: %d, Period 9: %d",
                    period1, period2, period3, period4, period5, period6, period7, period8, period9);

        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        mdl_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        mdl_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        mdl_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        mdl_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        mdl_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);
        mdl_sync_.setInterMessageLowerBound(7, PeriodBase * 0.001 * period8);
        mdl_sync_.setInterMessageLowerBound(8, PeriodBase * 0.001 * period9);
        
        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic9()
    {
    }

    private:
    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7, const sensor_msgs::msg::JointState::ConstSharedPtr& msg8, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg9)
    {
        rclcpp::Time output_time = this->now();

        if(count_mdl_ >= num_published_set_)
        {
            cout << "Approximate Time Model has got enough published sets !" << endl;

            outfile_mdl_ << observed_bcp1_ << " " << observed_bcp2_ << " " << observed_bcp3_ << " " << observed_bcp4_ << " " << observed_bcp5_ << " " << observed_bcp6_ << " " << observed_bcp7_ << " " << observed_bcp8_ << " " << observed_bcp9_ << " ";
            outfile_mdl_ << observed_wcp1_ << " " << observed_wcp2_ << " " << observed_wcp3_ << " " << observed_wcp4_ << " " << observed_wcp5_ << " " << observed_wcp6_ << " " << observed_wcp7_ << " " << observed_wcp8_ << " " << observed_wcp9_ << " ";
            outfile_mdl_ << min_trans_delay1_ /  Mstos << " " << min_trans_delay2_ /  Mstos << " " << min_trans_delay3_ /  Mstos << " " << min_trans_delay4_ /  Mstos << " " << min_trans_delay5_ /  Mstos << " " << min_trans_delay6_ /  Mstos << " " << min_trans_delay7_ /  Mstos << " " << min_trans_delay8_ /  Mstos << " " << min_trans_delay9_ /  Mstos << " ";
            outfile_mdl_ << max_trans_delay1_ /  Mstos << " " << max_trans_delay2_ /  Mstos << " " << max_trans_delay3_ /  Mstos << " " << max_trans_delay4_ /  Mstos << " " << max_trans_delay5_ /  Mstos << " " << max_trans_delay6_ /  Mstos << " " << max_trans_delay7_ /  Mstos << " " << max_trans_delay8_ /  Mstos << " " << max_trans_delay9_ /  Mstos << " ";
            outfile_mdl_ << max_passing_latency1_ << " " << max_passing_latency2_ << " " << max_passing_latency3_ << " " << max_passing_latency4_ << " " << max_passing_latency5_ << " " << max_passing_latency6_ << " " << max_passing_latency7_ << " " << max_passing_latency8_ << " " << max_passing_latency9_ << " ";
            outfile_mdl_ << max_reaction_latency1_ << " " << max_reaction_latency2_ << " " << max_reaction_latency3_ << " " << max_reaction_latency4_ << " " << max_reaction_latency5_ << " " << max_reaction_latency6_ << " " << max_reaction_latency7_ << " " << max_reaction_latency8_ << " " << max_reaction_latency9_ << endl;

            kill(getpid(),SIGINT);
            return;    
        }
        count_mdl_++;

        if (count_mdl_ > 1)
        {
            double reaction_latency1 = (output_time - previous_arrival_time1_).seconds();
            if (reaction_latency1 > max_reaction_latency1_)
            {
                max_reaction_latency1_ = reaction_latency1;
            }

            double reaction_latency2 = (output_time - previous_arrival_time2_).seconds();
            if (reaction_latency2 > max_reaction_latency2_)
            {
                max_reaction_latency2_ = reaction_latency2;
            }

            double reaction_latency3 = (output_time - previous_arrival_time3_).seconds();
            if (reaction_latency3 > max_reaction_latency3_)
            {
                max_reaction_latency3_ = reaction_latency3;
            }

            double reaction_latency4 = (output_time - previous_arrival_time4_).seconds();
            if (reaction_latency4 > max_reaction_latency4_)
            {
                max_reaction_latency4_ = reaction_latency4;
            }

            double reaction_latency5 = (output_time - previous_arrival_time5_).seconds();
            if (reaction_latency5 > max_reaction_latency5_)
            {
                max_reaction_latency5_ = reaction_latency5;
            }

            double reaction_latency6 = (output_time - previous_arrival_time6_).seconds();
            if (reaction_latency6 > max_reaction_latency6_)
            {
                max_reaction_latency6_ = reaction_latency6;
            }

            double reaction_latency7 = (output_time - previous_arrival_time7_).seconds();
            if (reaction_latency7 > max_reaction_latency7_)
            {
                max_reaction_latency7_ = reaction_latency7;
            }

            double reaction_latency8 = (output_time - previous_arrival_time8_).seconds();
            if (reaction_latency8 > max_reaction_latency8_)
            {
                max_reaction_latency8_ = reaction_latency8;
            }

            double reaction_latency9 = (output_time - previous_arrival_time9_).seconds();
            if (reaction_latency9 > max_reaction_latency9_)
            {
                max_reaction_latency9_ = reaction_latency9;
            }
        }

        previous_arrival_time1_ = msg1->timing.arrival_time;
        previous_arrival_time2_ = msg2->timing.arrival_time;
        previous_arrival_time3_ = msg3->timing.arrival_time;
        previous_arrival_time4_ = msg4->timing.arrival_time;
        previous_arrival_time5_ = msg5->timing.arrival_time;
        previous_arrival_time6_ = msg6->timing.arrival_time;
        previous_arrival_time7_ = msg7->timing.arrival_time;
        previous_arrival_time8_ = msg8->timing.arrival_time;
        previous_arrival_time9_ = msg9->timing.arrival_time;
        
        // Passing latency logging
        rclcpp::Time start_time1 = msg1->timing.arrival_time;
        rclcpp::Time start_time2 = msg2->timing.arrival_time;
        rclcpp::Time start_time3 = msg3->timing.arrival_time;
        rclcpp::Time start_time4 = msg4->timing.arrival_time;
        rclcpp::Time start_time5 = msg5->timing.arrival_time;
        rclcpp::Time start_time6 = msg6->timing.arrival_time;
        rclcpp::Time start_time7 = msg7->timing.arrival_time;
        rclcpp::Time start_time8 = msg8->timing.arrival_time;
        rclcpp::Time start_time9 = msg9->timing.arrival_time;

        double latency1 = (output_time - start_time1).seconds();
        double latency2 = (output_time - start_time2).seconds();
        double latency3 = (output_time - start_time3).seconds();
        double latency4 = (output_time - start_time4).seconds();
        double latency5 = (output_time - start_time5).seconds();
        double latency6 = (output_time - start_time6).seconds();
        double latency7 = (output_time - start_time7).seconds();
        double latency8 = (output_time - start_time8).seconds();
        double latency9 = (output_time - start_time9).seconds();

        if (latency1 > max_passing_latency1_)
        {
            max_passing_latency1_ = latency1;
        }

        if (latency2 > max_passing_latency2_)
        {
            max_passing_latency2_ = latency2;
        }

        if (latency3 > max_passing_latency3_)
        {
            max_passing_latency3_ = latency3;
        }

        if (latency4 > max_passing_latency4_)
        {
            max_passing_latency4_ = latency4;
        }

        if (latency5 > max_passing_latency5_)
        {
            max_passing_latency5_ = latency5;
        }

        if (latency6 > max_passing_latency6_)
        {
            max_passing_latency6_ = latency6;
        }

        if (latency7 > max_passing_latency7_)
        {
            max_passing_latency7_ = latency7;
        }

        if (latency8 > max_passing_latency8_)
        {
            max_passing_latency8_ = latency8;
        }

        if (latency9 > max_passing_latency9_)
        {
            max_passing_latency9_ = latency9;
        }
    }

    void callback1(sensor_msgs::msg::JointState::SharedPtr msg)
    {       
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous1_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<0>(msg);

        if (delay_previous1_ > max_trans_delay1_)
        {
            max_trans_delay1_ = delay_previous1_;
        }

        if (delay_previous1_ < min_trans_delay1_)
        {
            min_trans_delay1_ = delay_previous1_;
        }

        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }

                if(observed_now < observed_bcp1_)
                {
                    observed_bcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous2_ = absolute_delay.seconds() * Mstos;   

        mdl_sync_.add<1>(msg);

        if (delay_previous2_ > max_trans_delay2_)
        {
            max_trans_delay2_ = delay_previous2_;
        }

        if (delay_previous2_ < min_trans_delay2_)
        {
            min_trans_delay2_ = delay_previous2_;
        }

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }

                if(observed_now < observed_bcp2_)
                {
                    observed_bcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }
    }

    void callback3(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous3_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<2>(msg);

        if (delay_previous3_ > max_trans_delay3_)
        {
            max_trans_delay3_ = delay_previous3_;
        }

        if (delay_previous3_ < min_trans_delay3_)
        {
            min_trans_delay3_ = delay_previous3_;
        }

        double topic3_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp3_ > 0)
            {
                double observed_now = topic3_timestamp - previous_timestamp3_;
                if(observed_now > observed_wcp3_)
                {
                    observed_wcp3_ = observed_now;
                }

                if(observed_now < observed_bcp3_)
                {
                    observed_bcp3_ = observed_now;
                }
            }
            previous_timestamp3_ = topic3_timestamp;
        }
    }

    void callback4(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous4_ = absolute_delay.seconds() * Mstos; 

        mdl_sync_.add<3>(msg);

        if (delay_previous4_ > max_trans_delay4_)
        {
            max_trans_delay4_ = delay_previous4_;
        }

        if (delay_previous4_ < min_trans_delay4_)
        {
            min_trans_delay4_ = delay_previous4_;
        }

        double topic4_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp4_ > 0)
            {
                double observed_now = topic4_timestamp - previous_timestamp4_;
                if(observed_now > observed_wcp4_)
                {
                    observed_wcp4_ = observed_now;
                }

                if(observed_now < observed_bcp4_)
                {
                    observed_bcp4_ = observed_now;
                }
            }
            previous_timestamp4_ = topic4_timestamp;
        }
    }

    void callback5(sensor_msgs::msg::JointState::SharedPtr msg)
    {      
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous5_ = absolute_delay.seconds() * Mstos;
            
        mdl_sync_.add<4>(msg);

        if (delay_previous5_ > max_trans_delay5_)
        {
            max_trans_delay5_ = delay_previous5_;
        }

        if (delay_previous5_ < min_trans_delay5_)
        {
            min_trans_delay5_ = delay_previous5_;
        }

        double topic5_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp5_ > 0)
            {
                double observed_now = topic5_timestamp - previous_timestamp5_;
                if(observed_now > observed_wcp5_)
                {
                    observed_wcp5_ = observed_now;
                }

                if(observed_now < observed_bcp5_)
                {
                    observed_bcp5_ = observed_now;
                }
            }
            previous_timestamp5_ = topic5_timestamp;
        }
    }

    void callback6(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous6_ = absolute_delay.seconds() * Mstos;

        mdl_sync_.add<5>(msg);

        if (delay_previous6_ > max_trans_delay6_)
        {
            max_trans_delay6_ = delay_previous6_;
        }

        if (delay_previous6_ < min_trans_delay6_)
        {
            min_trans_delay6_ = delay_previous6_;
        }

        double topic6_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp6_ > 0)
            {
                double observed_now = topic6_timestamp - previous_timestamp6_;
                if(observed_now > observed_wcp6_)
                {
                    observed_wcp6_ = observed_now;
                }

                if(observed_now < observed_bcp6_)
                {
                    observed_bcp6_ = observed_now;
                }
            }
            previous_timestamp6_ = topic6_timestamp;
        }
    }

    void callback7(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous7_ = absolute_delay.seconds() * Mstos;  

        mdl_sync_.add<6>(msg);

        if (delay_previous7_ > max_trans_delay7_)
        {
            max_trans_delay7_ = delay_previous7_;
        }

        if (delay_previous7_ < min_trans_delay7_)
        {
            min_trans_delay7_ = delay_previous7_;
        }

        double topic7_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp7_ > 0)
            {
                double observed_now = topic7_timestamp - previous_timestamp7_;
                if(observed_now > observed_wcp7_)
                {
                    observed_wcp7_ = observed_now;
                }

                if(observed_now < observed_bcp7_)
                {
                    observed_bcp7_ = observed_now;
                }
            }
            previous_timestamp7_ = topic7_timestamp;
        }
    }

    void callback8(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous8_ = absolute_delay.seconds() * Mstos;      

        mdl_sync_.add<7>(msg);

        if (delay_previous8_ > max_trans_delay8_)
        {
            max_trans_delay8_ = delay_previous8_;
        }

        if (delay_previous8_ < min_trans_delay8_)
        {
            min_trans_delay8_ = delay_previous8_;
        }

        double topic8_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp8_ > 0)
            {
                double observed_now = topic8_timestamp - previous_timestamp8_;
                if(observed_now > observed_wcp8_)
                {
                    observed_wcp8_ = observed_now;
                }

                if(observed_now < observed_bcp8_)
                {
                    observed_bcp8_ = observed_now;
                }
            }
            previous_timestamp8_ = topic8_timestamp;
        }
    }

    void callback9(sensor_msgs::msg::JointState::SharedPtr msg)
    {      
        msg->timing.arrival_time = this->now();

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous9_ = absolute_delay.seconds() * Mstos;      

        mdl_sync_.add<8>(msg);

        if (delay_previous9_ > max_trans_delay9_)
        {
            max_trans_delay9_ = delay_previous9_;
        }

        if (delay_previous9_ < min_trans_delay9_)
        {
            min_trans_delay9_ = delay_previous9_;
        }

        double topic9_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_mdl_ < num_published_set_)
        {
            if(previous_timestamp9_ > 0)
            {
                double observed_now = topic9_timestamp - previous_timestamp9_;
                if(observed_now > observed_wcp9_)
                {
                    observed_wcp9_ = observed_now;
                }

                if(observed_now < observed_bcp9_)
                {
                    observed_bcp9_ = observed_now;
                }
            }
            previous_timestamp9_ = topic9_timestamp;
        }
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_, topic6_sub_, topic7_sub_, topic8_sub_, topic9_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_, period3_, period4_, period5_, period6_, period7_, period8_, period9_;

    int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets
        
    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_, delay_previous3_, delay_previous4_, delay_previous5_, delay_previous6_, delay_previous7_, delay_previous8_, delay_previous9_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_, previous_timestamp4_, previous_timestamp5_, previous_timestamp6_, previous_timestamp7_, previous_timestamp8_, previous_timestamp9_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_, observed_wcp4_, observed_wcp5_, observed_wcp6_, observed_wcp7_, observed_wcp8_, observed_wcp9_;

    double observed_bcp1_ = FLT_MAX, observed_bcp2_ = FLT_MAX, observed_bcp3_ = FLT_MAX, observed_bcp4_ = FLT_MAX, observed_bcp5_ = FLT_MAX, observed_bcp6_ = FLT_MAX, observed_bcp7_ = FLT_MAX, observed_bcp8_ = FLT_MAX, observed_bcp9_ = FLT_MAX;

    double max_trans_delay1_ = 0., max_trans_delay2_ = 0., max_trans_delay3_ = 0., max_trans_delay4_ = 0., max_trans_delay5_ = 0., max_trans_delay6_ = 0., max_trans_delay7_ = 0., max_trans_delay8_ = 0., max_trans_delay9_ = 0.;
    double min_trans_delay1_ = FLT_MAX, min_trans_delay2_ = FLT_MAX, min_trans_delay3_ = FLT_MAX, min_trans_delay4_ = FLT_MAX, min_trans_delay5_ = FLT_MAX, min_trans_delay6_ = FLT_MAX, min_trans_delay7_ = FLT_MAX, min_trans_delay8_ = FLT_MAX, min_trans_delay9_ = FLT_MAX;
    double max_passing_latency1_ = 0., max_passing_latency2_ = 0., max_passing_latency3_ = 0., max_passing_latency4_ = 0., max_passing_latency5_ = 0., max_passing_latency6_ = 0., max_passing_latency7_ = 0., max_passing_latency8_ = 0., max_passing_latency9_ = 0.;

    double max_reaction_latency1_ = 0., max_reaction_latency2_ = 0., max_reaction_latency3_ = 0., max_reaction_latency4_ = 0., max_reaction_latency5_ = 0., max_reaction_latency6_ = 0., max_reaction_latency7_ = 0., max_reaction_latency8_ = 0., max_reaction_latency9_ = 0.;
    rclcpp::Time previous_arrival_time1_, previous_arrival_time2_, previous_arrival_time3_, previous_arrival_time4_, previous_arrival_time5_, previous_arrival_time6_, previous_arrival_time7_, previous_arrival_time8_, previous_arrival_time9_;
};
}
