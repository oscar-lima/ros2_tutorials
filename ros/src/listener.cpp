/*
 * Copyright [2018] <Instituto Superior Tecnico>
 *
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 * 
 * ROS 2 chatter tutorial (only listener is coded here)
 * 
 */

#include <ros2_tutorials/listener.h>

ListenerNode::ListenerNode(): Node("listener"), node_frequency_(0.0), is_chatter_msg_received_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing node...");

    // setup subscriber
    chatter_sub_ = this->create_subscription<std_msgs::msg::String>("chatter", std::bind(&ListenerNode::chatterCallBack, this, _1));
    
    // init parameter handler
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
    
    // get node params and store in member variables
    get_params();

    RCLCPP_INFO(this->get_logger(), "Node initialized...");
}

ListenerNode::~ListenerNode()
{
    rclcpp::shutdown();
}

void ListenerNode::get_params()
{
    // log to console
    RCLCPP_INFO(this->get_logger(), "Getting params");
    
    // waiting for parameter service to become available
    if (!parameters_client_->wait_for_service(1s))
    {
        // parameter not available, set default value
        RCLCPP_INFO(this->get_logger(), "Parameter not available, setting default...")

        // set default value for parameter
        node_frequency_ = 10.0;
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

void ListenerNode::chatterCallBack(const std_msgs::msg::String::SharedPtr msg)
{
    chat_msg_ = msg;
    is_chatter_msg_received_ = true;
}

void ListenerNode::update()
{
    if (is_chatter_msg_received_)
    {
        // lower flag
        is_chatter_msg_received_ = false;

        // display msg content
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", chat_msg_->data.c_str());
    }
}

int main(int argc, char **argv)
{
    // init node
    rclcpp::init(argc, argv);

    // declare empty executor
    rclcpp::executors::SingleThreadedExecutor executor;

    // create object of ListenerNode class as shared pointer
    std::shared_ptr<ListenerNode> listener_node = std::make_shared<ListenerNode>();

    // add node to executor
    executor.add_node(listener_node);

    // set the rate at which node will run
    rclcpp::Rate loop_rate(listener_node->node_frequency_);

    while (rclcpp::ok())
    {
        // listen to callbacks
        executor.spin_once(1ms);

        // main loop function
        listener_node->update();

        // sleep to control the node frequency
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
