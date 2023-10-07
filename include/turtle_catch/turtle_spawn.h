#pragma once
#ifndef TURTLE_CATCH_TURTLE_SPAWN_H
#define TURTLE_CATCH_TURTLE_SPAWN_H

// ros 2 lib
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>

// standard lib
#include <random>

class TurtleSpawn : public rclcpp::Node
{
public:
    TurtleSpawn();

private:
    double lower_bound_;
    double upper_bound_;
    std::random_device random_device_;
    std::mt19937 generator_;
    std::uniform_real_distribution<double> random_location_; 
    std::uniform_real_distribution<double> random_angle_; 

private:
    std::thread spawn_turtle_thread_;
    std::vector<std::thread> spawn_turtle_vec_thread_;
    int count_;

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_turtle_client_;
    rclcpp::TimerBase::SharedPtr vector_thread_timer_;
    void init_all();
    void call_spawn_turtle_service(float x, float y, float theta, std::string name);

    void spawn_turtle_callback();
    void vector_thread_timer_callback();
};

#endif