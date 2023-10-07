#include "turtle_catch/turtle_spawn.h"

TurtleSpawn::TurtleSpawn()
    : Node("turtle_spawn_cpp"),
      lower_bound_(00.00),
      upper_bound_(11.00),
      generator_(random_device_()),
      random_location_(lower_bound_, upper_bound_),
      random_angle_(0.0, 2 * M_PI),
      count_(1)
{

    init_all();
    RCLCPP_INFO(this->get_logger(),
                "object of TurtleSpawn class has been created.");

    /*--------------test-1-----------------*/
    /*
    spawn_turtle_thread_ = std::thread(
        std::bind(&TurtleSpawn::spawn_turtle_callback, this));
    */

    /*--------------test-2-----------------*/
    /*
    for (int i = 0; i < 5; i++)
    {
        spawn_turtle_vec_thread_.push_back(
            std::thread(
                std::bind(&TurtleSpawn::spawn_turtle_callback, this)));
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
    */
}

void TurtleSpawn::init_all()
{
    spawn_turtle_client_ = this->create_client<turtlesim::srv::Spawn>(
        "spawn");
    vector_thread_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                                   std::bind(&TurtleSpawn::vector_thread_timer_callback, this));
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

void TurtleSpawn::spawn_turtle_callback()
{
    count_++;

    double x = random_location_(generator_);
    double y = random_location_(generator_);
    double theta = random_angle_(generator_);
    std::string name = "turtle" + std::to_string(count_);

    call_spawn_turtle_service(x, y, theta, name);
}

void TurtleSpawn::vector_thread_timer_callback()
{
    spawn_turtle_vec_thread_.push_back(
        std::thread(
            std::bind(&TurtleSpawn::spawn_turtle_callback, this)));
}
