
#include "nodes/camera_node.hpp"
#include "nodes/imu_node.hpp"
#include "nodes/io_node.hpp"
#include "nodes/lidar_node.hpp"
#include "nodes/line_node.hpp"
#include "nodes/motor_node.hpp"
#include "loops/corridor_loop.hpp"
#include "loops/line_loop.hpp"


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    /******************* LINE FOLLOWING ************************
    // Create multiple nodes
    auto node1 = std::make_shared<nodes::MotorNode>();
    auto node2 = std::make_shared<nodes::LineNode>();
    auto node3 = std::make_shared<nodes::IoNode>();

    // Loop node
    auto loop_node = std::make_shared<nodes::LineLoop>();
    loop_node->init(node1,node2); // Add nodes to loop

    // Add nodes to the executor
    executor->add_node(node1);
    executor->add_node(node2);
    executor->add_node(node3);
    ************************************************************/

    /**************** CORRIDOR LOOP ****************************/
    // Create multiple nodes
    auto node1 = std::make_shared<nodes::LidarNode>();
    auto node2 = std::make_shared<nodes::MotorNode>();
    auto node3 = std::make_shared<nodes::ImuNode>();
    auto node4 = std::make_shared<nodes::CameraNode>();

    // Loop node
    auto loop_node = std::make_shared<nodes::CorridorLoop>();
    loop_node->init(node2,node3,node1,node4); // Add nodes to loop

    // Add nodes to the executor
    executor->add_node(node1);
    executor->add_node(node2);
    executor->add_node(node3);
    executor->add_node(node4);
    /***********************************************************/

    // Add loop node to the executor
    executor->add_node(loop_node);

    // Run the executor (handles callbacks for all nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
