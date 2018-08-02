/*
 * Copyright [2018] <Instituto Superior Tecnico>
 *
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 * 
 * ROS 2 chatter tutorial (only talker is covered here)
 * 
 */

#ifndef CHATTER_TALKER_H
#define CHATTER_TALKER_H

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class TalkerNode : public rclcpp::Node
{
  public:
    // constructor
    TalkerNode();

    // descructor
    ~TalkerNode();

    // get node params
    void get_params();

    // node core function
    void update();

    // node main loop
    void start_node();

  private:
    // publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_pub_;

    // to query from param server
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;

    // to store node parameters
    std::shared_future<std::vector<rclcpp::parameter::ParameterVariant> > parameters_;

    // example where an integer get incremented and published
    int count_;

    // the frequency at which the node will run
    double node_frequency_;
};

#endif  // CHATTER_TALKER_H
