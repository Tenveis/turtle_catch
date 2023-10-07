#include "turtle_catch/turtle_spawn.h" 

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TurtleSpawn>();
    RCLCPP_INFO(node->get_logger(), "turtle_spawn_node is initialized.");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}