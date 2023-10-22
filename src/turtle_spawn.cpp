#include "turtle_catch/turtle_spawn.h"

TurtleSpawn::TurtleSpawn()
    : Node("turtle_spawn_cpp"),
      lower_bound_(00.00),
      upper_bound_(11.00),
      generator_(random_device_()),
      random_location_(lower_bound_, upper_bound_),
      random_angle_(0.0, 2 * M_PI),
      turtle_count_(1)
{

    init_all();
    RCLCPP_INFO(this->get_logger(),
                "object of TurtleSpawn class has been created.");
}

void TurtleSpawn::init_all()
{
    spawn_turtle_client_ = this->create_client<turtlesim::srv::Spawn>(
        "spawn");
    alive_turtles_pub_ = this->create_publisher<turtle_catch::msg::TurtleArray>("alive_turtles", 100);
    catch_turtle_server_ = this->create_service<turtle_catch::srv::CatchTurtle>(
        "catch_turtle",
        std::bind(&TurtleSpawn::catch_turtle_serverCallbacK, this, std::placeholders::_1, std::placeholders::_2));
    spawn_turtle_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&TurtleSpawn::spawn_turtle_timer_callback, this));
}

void TurtleSpawn::call_spawn_turtle_service(double x, double y, double theta, std::string name)
{
    while (!spawn_turtle_client_->wait_for_service(std::chrono::milliseconds(1000)))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Waiting for service server: /spawn");
    }

    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = x;
    request->y = y;
    request->theta = theta;
    request->name = name;

    auto future = spawn_turtle_client_->async_send_request(request);
    try
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(),
                    "%s is spawned.", response.get()->name.c_str());
        turtle_catch::msg::Turtle spawned_turtle;
        spawned_turtle.x = x;
        spawned_turtle.y = y;
        spawned_turtle.theta = theta;
        spawned_turtle.name = name;

        alive_turtles_.push_back(spawned_turtle);
        publish_alive_turtles();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR_STREAM(
            this->get_logger(),
            request->name.c_str()
                << " is not spawned.\n"
                << "ERROR: " << e.what());
    }
}

void TurtleSpawn::publish_alive_turtles()
{
    auto msg = turtle_catch::msg::TurtleArray();

    for (auto turtle : alive_turtles_)
    {
        msg.turtles.push_back(turtle);
    }
    alive_turtles_pub_->publish(msg);
}

void TurtleSpawn::catch_turtle_serverCallbacK(const turtle_catch::srv::CatchTurtle::Request::SharedPtr req,
                                              const turtle_catch::srv::CatchTurtle::Response::SharedPtr res)

{
    kill_turtle_vec_thread_.push_back(
        std::thread(
            std::bind(
                &TurtleSpawn::call_turteKillService, this, req->name)));

    res->success = true;
}

void TurtleSpawn::call_turteKillService(std::string name)
{
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client;
    client = this->create_client<turtlesim::srv::Kill>("kill");

    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "waiting for service server: /kill");
    }
    auto req = std::make_shared<turtlesim::srv::Kill::Request>();
    req->name = name;

    auto future = client->async_send_request(req);
    try
    {
        auto response = future.get();
        for (auto turtle : alive_turtles_)
        {
            try
            {
                if (name == turtle.name)
                {
                    alive_turtles_.remove(turtle);
                    publish_alive_turtles();
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';

                RCLCPP_ERROR_STREAM(
                    this->get_logger(),
                    "Failed to remove...\n"
                        << "ERROR: " << e.what());
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';

        RCLCPP_ERROR_STREAM(
            this->get_logger(),
            " Service call failed.\n"
                << "ERROR: " << e.what());
    }
}

void TurtleSpawn::spawn_turtle_timer_callback()
{
    turtle_count_++;

    double x = random_location_(generator_);
    double y = random_location_(generator_);
    double theta = random_angle_(generator_);
    std::string name = "turtle" + std::to_string(turtle_count_);

    spawn_turtle_vec_thread_.push_back(
        std::thread(
            std::bind(&TurtleSpawn::call_spawn_turtle_service, this, x, y, theta, name)));
}
