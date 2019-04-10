#include "openrover.hpp"

#include "rclcpp/rclcpp.hpp"
#include "openrover_serial.hpp"
#include "openrover_node.hpp"
#include <string>

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::map<std::string, std::string> argmap;
	std::vector<rclcpp::Parameter> params;

	for (auto i = 1; i<argc ;i++) {
		std::string arg = argv[i];
		std::cout << "got arg: %s"<< arg<<std::endl;
		std::string delim = ":=";
		auto pos = arg.find(delim);
		if (pos >= arg.size()) {
			std::cout << "could not process argument" << arg << std::endl;
		}
		else {
			auto k = arg.substr(0, pos);
			auto v = arg.substr(pos + delim.size());
			argmap.emplace(k, v);
		}
	}
	
	std::string port;
	auto port_kv = argmap.find("port");
	if (port_kv != argmap.end()) {
		port = port_kv->second;
		std::cout << "Port specified: " << port<< std::endl;
	}
	else {
		std::cout << "No port specified. Searching." <<std::endl;
		auto ports = OpenRoverSerial::list_ftdi_ports();
		if (ports.size() == 0) {
			std::cout << "No ports found" << std::endl;
			exit(1);
		}
		std::cout << "Found ports:" << std::endl;
		for (auto p : ports) {
			std::cout << "\t" << p << std::endl;
		}
		port = ports[0];
	}

	rclcpp::executors::SingleThreadedExecutor executor;
	
	auto n1 = std::make_shared<OpenRoverSerial>(port);
	executor.add_node(n1);

	auto n2 = std::make_shared<OpenRoverNode>();
	executor.add_node(n2);
	executor.spin();
	return 0;
}