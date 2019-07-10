#include "composition.hpp"

#include <string>
#include "rover.hpp"
#include "rover_serial.hpp"
using namespace openrover;

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  auto rover_serial_node = std::make_shared<RoverSerial>();
  executor.add_node(rover_serial_node);

  auto rover_node = std::make_shared<Rover>();
  executor.add_node(rover_node);

  executor.spin();
  return 0;
}
