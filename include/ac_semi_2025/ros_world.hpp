#error "お気持ちコード。動かないのでちゃんと実装してね"

#pragma once

#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace ac_semi_2025::ros_world::impl {
	using Eigen::Matrix2Xd;

	struct RosWorld final : rclcpp::Node {
		Matrix2Xd laserscan{};
		rclcpp::Publisher<void/** @todo 自作のメッセージ型を推奨 */>::SharePtr robot_speed_pub;
		rclcpp::Subscriber<sensor_msgs::msg::LaserScan>::SharePtr lidar_sub;

		Ros2Node():
			rclcpp::Node{/** @todo */}
			, robot_speed_pub{/** @todo */}
			, lidar_sub{/** @todo */}
		{}

		auto update(const /** @todo */ auto& robot_speed) const noexcept -> Matrix2Xd {
			/// @todo 以下あてにならん
			const auto robot_speed_msg = robot_speed;
			this->robot_speed_pub->publish(robot_speed_msg);
			return this->laserscan;
		}

		/// @todo 引数の型おかしいかも
		void laserscan_callback(const sensor_msgs::msg::laserScan::ConstSharedPtr msg) noexcept {
			/// @todo 以下あてにならん
			this->laserscan = msg;
		}
	};
}

namespace ac_semi_2025::ros_world {
	using impl::RosWorld;
}