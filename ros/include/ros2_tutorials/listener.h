/*
 * Copyright [2020] <GPLv3>
 *
 * Author: Oscar Lima (oscar.lima@dfki.de)
 * 
 * ROS 2 chatter tutorial (only talker is covered here)
 * 
 */

#ifndef ROS2_TUTORIALS_LISTENER_H
#define ROS2_TUTORIALS_LISTENER_H

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// for subscriber
using namespace std;
using std::placeholders::_1;

// for parameter service 1s wait timeout
using namespace std::chrono_literals;

class ListenerNode : public rclcpp::Node
{
  public:
    // constructor
    ListenerNode();

    // destructor
    ~ListenerNode();

    // get node params
    void get_params();

    // callback for incoming string msgs
    void chatterCallBack(const std_msgs::msg::String::SharedPtr msg);

    // node core function
    void update();

    // node main loop
    void start_node();

    // the frequency at which the node will run
    double node_frequency_;

  private:
    // Subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chatter_sub_;

    // to query from param server
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;

    // to store the incoming msg from callback
    std_msgs::msg::String::SharedPtr chat_msg_;

    // flag to indicate that msg was received
    bool is_chatter_msg_received_;
};

#endif  // ROS2_TUTORIALS_LISTENER_H
