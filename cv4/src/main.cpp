#include <oneapi/tbb/partitioner.h>

#include "example.hpp"
#include "nodes/io_nodes.hpp"
#include "nodes/motor_nodes.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);


    rclcpp::spin(std::make_shared<nodes::MotorNode>());
    rclcpp::shutdown();
    return 0;
}
