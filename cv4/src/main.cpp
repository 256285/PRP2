//#include <oneapi/tbb/partitioner.h>

#include "example.hpp"
#include "loops/line_loop.hpp"
#include "nodes/io_nodes.hpp"
#include "nodes/lidar_node.hpp"
#include "nodes/line_sens.hpp"
#include "nodes/motor_nodes.hpp"
#include "loops/corridor_loop.hpp"
#include "nodes/Camera_node.hpp"
#include "nodes/imu_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);


    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create multiple nodes
    auto node1 = std::make_shared<nodes::LidarNode>();
    auto node2 = std::make_shared<nodes::MotorNode>();
    auto node3 = std::make_shared<nodes::ImuNode>();
    auto node4 = std::make_shared<nodes::CorridorLoop>();
    //auto node5 = std::make_shared<nodes::CameraNode>();
    node4->init(node2,node3,node1);


    // Add nodes to the executor
    executor->add_node(node1);
    executor->add_node(node2);
    executor->add_node(node3);
    executor->add_node(node4);
    //executor->add_node(node5);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
