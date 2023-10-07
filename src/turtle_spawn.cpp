#include "turtle_catch/turtle_spawn.h"

TurtleSpawn::TurtleSpawn()
    : Node("turtle_spawn_cpp"), count_(1)
{
    init_all();
    RCLCPP_INFO(this->get_logger(),
                "object of TurtleSpawn class has been created.");

    /*--------------test-----------------*/
    spawn_turtle_thread_ = std::thread(
        std::bind(&TurtleSpawn::spawn_turtle_timer_callback, this));

    /*--------------test-2-----------------*/
    for (int i = 0; i < 20; i++)
    {
        thread_.push_back(
            std::thread(
                std::bind(&TurtleSpawn::spawn_turtle_timer_callback, this)));
    }
}

void TurtleSpawn::init_all()
{
    spawn_turtle_client_ = this->create_client<turtlesim::srv::Spawn>(
        "spawn");
}

void TurtleSpawn::call_spawn_turtle_service(float x, float y, float theta, std::string name)
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

void TurtleSpawn::spawn_turtle_timer_callback()
{
    count_++;
    std::string name = "turtle" + std::to_string(count_);
    float x = 1.1 + (2 * count_ / 10);
    float y = 3.3 + (2 * count_ / 10);
    float theta = 0.0 + (10 * count_);

    call_spawn_turtle_service(x, y, theta, name);
}
