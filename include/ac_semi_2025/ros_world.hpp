#pragma once

#include <cmath>
#include <utility>
#include <memory>
#include <mutex>
#include <condition_variable>

#include <eigen3/Eigen/Dense>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ac_semi_2025/msg/pose2d.hpp>

#include "utility.hpp"
#include "geometry.hpp"

namespace ac_semi_2025::ros_world::impl {
	using Eigen::Matrix2Xd;
	using Eigen::Vector2d;

	using geometry::Pose2d;

	struct RosWorld final : rclcpp::Node {
		std::unique_ptr<Matrix2Xd> laserscan{};
		std::condition_variable condvar{};
		std::mutex mtx{};
		rclcpp::Publisher<ac_semi_2025::msg::Pose2d>::SharedPtr robot_speed_pub;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
		rclcpp::Publisher<ac_semi_2025::msg::Pose2d>::SharedPtr pose_pub;

		RosWorld():
			rclcpp::Node{"ros_world"}
			, robot_speed_pub{this->create_publisher<ac_semi_2025::msg::Pose2d>("robot_speed", 10)}
			, lidar_sub{this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) -> void {
				this->laserscan_callback(msg);
			})}
			, pose_pub{this->create_publisher<ac_semi_2025::msg::Pose2d>("icped_pose", 10)}
		{}

		virtual ~RosWorld() override = default;

		auto update(const Pose2d& robot_speed, const double) noexcept -> std::unique_ptr<Matrix2Xd> {
			ac_semi_2025::msg::Pose2d msg{};
			msg.x = robot_speed.xy(0);
			msg.y = robot_speed.xy(1);
			msg.th = robot_speed.th;
			this->robot_speed_pub->publish(msg);
			{
				std::unique_lock lck{this->mtx};
				this->condvar.wait(lck, [this] -> bool {
					return static_cast<bool>(this->laserscan);
				});
				return std::move(this->laserscan);
			}
		}

		void publish_pose(const Pose2d& pose) const noexcept {
			ac_semi_2025::msg::Pose2d msg{};
			msg.x = pose.xy(0);
			msg.y = pose.xy(1);
			msg.th = pose.th;
			this->pose_pub->publish(msg);
		}

		void laserscan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) noexcept {
			/// @todo 上と関連するが、これを非同期アクセス可能にすべきやも
			const auto& ranges = msg->ranges;
			const i64 n = ranges.size();
			const double angle_min = msg->angle_min;
			const double angle_increment = msg->angle_increment;
			Matrix2Xd laserscan{2, n};
			for(i64 i = 0; i < n; ++i) {
				const double th = angle_min + angle_increment * i;
				laserscan.col(i) = ranges[i] * Vector2d{std::cos(th), std::sin(th)};
			}
			{
				std::unique_lock lck{this->mtx};
				this->laserscan = std::make_unique<Matrix2Xd>(std::move(laserscan));
			}
		}
	};
}

namespace ac_semi_2025::ros_world {
	using impl::RosWorld;
}