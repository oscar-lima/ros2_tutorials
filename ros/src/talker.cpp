/*
 * Copyright [2018] <Instituto Superior Tecnico>
 *
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 * 
 * ROS 2 chatter tutorial (only talker is coded here)
 * 
 */

#include <chatter/talker.h>

TalkerNode::TalkerNode(): Node("talker"), count_(0), node_frequency_(0.0)
{
    RCLCPP_INFO(this->get_logger(), "Initializing node...");
    
    // create publisher
    chatter_pub_ = this->create_publisher<std_msgs::msg::String>("chatter");

    // init parameter handler
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
    
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
    
    // waiting for parameter service to become available
    if (!parameters_client_->wait_for_service(1s))
    {
        // parameter not available, set default value
        RCLCPP_INFO(this->get_logger(), "Parameter not available, setting default...")

        // set default value for parameter
        node_frequency_ = 2.0;
    }
    else
    {
        // parameter available, reading it
        RCLCPP_INFO(this->get_logger(), "Parameter available, setting from server");
    
        // get params
        auto parameters = parameters_client_->get_parameters({"node_frequency"});

        std::stringstream ss;
        for (auto & parameter : parameters.get())
        {
            if (parameter.get_name() == "node_frequency")
            {
                node_frequency_ = parameter.get_parameter_value().double_value;
            }
        }

        RCLCPP_INFO(this->get_logger(), ss.str().c_str())
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
