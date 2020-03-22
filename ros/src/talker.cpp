/*
 * Copyright [2020] <GPLv3>
 *
 * Author: Oscar Lima (oscar.lima@dfki.de)
 *
 * ROS 2 chatter tutorial (only talker is covered here)
 *
 */

#include <ros2_tutorials/talker.h>

TalkerNode::TalkerNode(): Node("talker"), count_(0), node_frequency_(15.0)
{
    RCLCPP_INFO(this->get_logger(), "Initializing node...");

    // create publisher with quality of service 10
    chatter_pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    // init parameter handler
    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);

    // get node params and store in member variables
    get_params();

    RCLCPP_INFO(this->get_logger(), "Node initialized...");
}

TalkerNode::~TalkerNode()
{
    rclcpp::shutdown();
}

void TalkerNode::get_params()
{
    // log to console
    RCLCPP_INFO(this->get_logger(), "Getting params");

    if (!parameters_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_WARN(this->get_logger(), "Parameter service not available, setting default params");
            node_frequency_ = 10.0;
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Parameter service available");

        // example: check parameter existance
        if(!parameters_client_->has_parameter("node_frequency"))
            RCLCPP_WARN(this->get_logger(), "Parameter node_frequency not found, setting default value");

        // get parameter, if not available set default value
        node_frequency_ = parameters_client_->get_parameter<double>("node_frequency", 5.0);
    }

    RCLCPP_INFO(this->get_logger(), "Node will run at : %lf [hz]", node_frequency_);
}

void TalkerNode::update()
{
    // create std_msgs string custom ROS2 msg
    std_msgs::msg::String msg;

    // fill msg
    msg.data = std::to_string(count_++);

    // publish msg
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    chatter_pub_->publish(msg);
}

void TalkerNode::start_node()
{
    // define the rate at which node will run
    rclcpp::Rate loop_rate(node_frequency_);

    while (rclcpp::ok())
    {
        // main loop function
        update();

        // sleep to control the node frequency
        loop_rate.sleep();

        // alternative to ROS1 ros::spinOnce()
        // you might wonder, why spin when this node has no callbacks (does not subscribe to anything)
        // well turns out parameters need this, if you do "ros2 param list" without spinning, it will hang
        // another option when you do not need to constantly publish stuff (like this node does), do a normal spin
        // rclcpp::spin(this->get_node_base_interface());
        rclcpp::spin_some(this->get_node_base_interface());
    }
}

int main(int argc, char **argv)
{
    // init node
    rclcpp::init(argc, argv);

    // create object of the node class (TalkerNode)
    TalkerNode talker_node;

    // start node !
    talker_node.start_node();

    return 0;
}
