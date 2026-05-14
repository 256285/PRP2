#pragma once
#include <rclcpp/rclcpp.hpp>

#include "algorithms/pid.hpp"
#include "nodes/camera_node.hpp"
#include "nodes/lidar_node.hpp"
#include "nodes/motor_node.hpp"
#include "nodes/imu_node.hpp"

namespace nodes {
    class CorridorLoop : public rclcpp::Node {
        rclcpp::TimerBase::SharedPtr timer_;    // timer
        // Used nodes for this loop
        std::shared_ptr<nodes::MotorNode> motor_node;
        std::shared_ptr<nodes::ImuNode> imu_node;
        std::shared_ptr<nodes::LidarNode> lidar_node;
        std::shared_ptr<nodes::CameraNode> camera_node;
    public:
        enum corridor_mode {
            CALIBRATION,
            CORRIDOR_FOLLOWING,
            BACK_FOLLOWING,
            STRAIGHT,
            STOP,
            TURN_L,
            TURN_R,
            TURN_BACK,
            WAIT
        };
        enum corridor_type {
            EIGHT,
            MAZE,
        };
        // Constructor
        CorridorLoop():Node("corridor_loop") {
            timer_ = this->create_wall_timer(
               std::chrono::milliseconds(static_cast<int>(1000.0 / 20)),
               std::bind(&CorridorLoop::state_machine, this));
        }
        // Destructor (default)
        ~CorridorLoop() override = default;
        // Initialize used nodes
        void init(std::shared_ptr<nodes::MotorNode> motor_nodeN, std::shared_ptr<nodes::ImuNode> imu_nodeN, std::shared_ptr<nodes::LidarNode> lidar_nodeN, std::shared_ptr<nodes::CameraNode> camera_nodeN) {
            motor_node = motor_nodeN;
            imu_node = imu_nodeN;
            lidar_node = lidar_nodeN;
            camera_node = camera_nodeN;
        }
        
    private:
        algorithms::Pid pid = algorithms::Pid (4.9,3.8,0);
        corridor_type current_type = corridor_type::EIGHT;
        corridor_mode mode = corridor_mode::CALIBRATION;                // current mode of state machine
        corridor_mode next_mode = corridor_mode::CORRIDOR_FOLLOWING;    // next mode after STOP
        nodes::CameraNode::Direction next_dir = nodes::CameraNode::Direction::RIGHT; // preferred direction
        //algorithms::Coordinates prev;
        float prev = 0;                                                 // previous distance
        float yaw_ref = 0;                                              // saved yaw
        float speed = 149;
        float turn_speed = 8;
        float prev_distance = 0;
        uint64_t time;
        // Full state machine for Maze
        void state_machine();

        // Pure PID algorithm, only usable for Corridor (not Maze)
        void PID();
    };
}
