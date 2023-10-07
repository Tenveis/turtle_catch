#pragma once
#ifndef TURTLE_CATCH_TURTLE_SPAWN_H
#define TURTLE_CATCH_TURTLE_SPAWN_H

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>

class TurtleSpawn : public rclcpp::Node
{
public:
    TurtleSpawn();

private:
    std::thread spawn_turtle_thread_;
    std::vector<std::thread> thread_;
 int count_;

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_turtle_client_;

    void init_all();
    void call_spawn_turtle_service(float x, float y, float theta, std::string name);

    void spawn_turtle_timer_callback();
};

#endif