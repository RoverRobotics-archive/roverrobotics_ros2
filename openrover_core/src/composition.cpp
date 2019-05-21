#include "composition.hpp"

#include "connection.hpp"
#include "rover.hpp"
#include <string>
using namespace openrover;

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  auto connection_node = std::make_shared<Connection>();
  executor.add_node(connection_node);

  auto rover_node = std::make_shared<Rover>();
  executor.add_node(rover_node);

  executor.spin();
  return 0;
}