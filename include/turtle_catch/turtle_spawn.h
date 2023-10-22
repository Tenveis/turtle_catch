#pragma once
#ifndef TURTLE_CATCH_TURTLE_SPAWN_H
#define TURTLE_CATCH_TURTLE_SPAWN_H

// ros 2 lib
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/kill.hpp>

// custom lib
#include "turtle_catch/msg/turtle.hpp"
#include "turtle_catch/msg/turtle_array.hpp"
#include "turtle_catch/srv/catch_turtle.hpp"

// standard lib
#include <random>
#include <list>

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
    std::vector<std::thread> spawn_turtle_vec_thread_;
    std::vector<std::thread> kill_turtle_vec_thread_;
    int turtle_count_;

    std::list<turtle_catch::msg::Turtle> alive_turtles_;

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_turtle_client_;
    rclcpp::Publisher<turtle_catch::msg::TurtleArray>::SharedPtr alive_turtles_pub_;
    rclcpp::Service<turtle_catch::srv::CatchTurtle>::SharedPtr catch_turtle_server_;
    rclcpp::TimerBase::SharedPtr spawn_turtle_timer_;

    void init_all();
    void call_spawn_turtle_service(double x, double y, double theta, std::string name);
    void publish_alive_turtles();
    void catch_turtle_serverCallbacK(const turtle_catch::srv::CatchTurtle::Request::SharedPtr req,
                                     const turtle_catch::srv::CatchTurtle::Response::SharedPtr res);
    
    void call_turteKillService(std::string name);
    void spawn_turtle_timer_callback();
};

#endif