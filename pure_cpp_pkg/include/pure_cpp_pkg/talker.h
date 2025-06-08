#ifndef HYBRID_PKG_TALKER_H
#define HYBRID_PKG_TALKER_H

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class TalkerNode : public rclcpp::Node
{
  public:
    // constructor
    TalkerNode();

    // destructor
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
    rclcpp::SyncParametersClient::SharedPtr parameters_client_;

    // example where an integer get incremented and published
    int count_;

    // the frequency at which the node will run
    double node_frequency_;
};

#endif  // HYBRID_PKG_TALKER_H
