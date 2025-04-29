#pragma once

#include <random>

#include <eigen3/Eigen/Dense>

#include "geometry.hpp"
#include "global_map.hpp"
#include "diff2_pure_pursuit.hpp"

namespace ac_semi_2025::simulator::impl {
	using Eigen::Matrix2Xd;
	using Eigen::Vector2d;

	using geometry::Pose2d;
	using geometry::Line2d;
	using geometry::Ray2d;
	using global_map::GlobalMap;
	using diff2_carrot_pursuit::Diff2wheelSpeed;

	/// @todo 実装
	struct SimulatorConstant final {
		GlobalMap<Line2d> map;
		double noise_gain_ahead;
		double noise_gain_rotate;
		i64 laser_count;
		double laser_angle_min;
		double laser_angle_range;
	};
	struct SimulatorState final {
		Pose2d pose;
		std::default_random_engine engine{};
		std::normal_distribution<double> dist{0.0, 1.0};

		static auto make(const Pose2d& init_pose) noexcept -> SimulatorState {
			return SimulatorState{init_pose};
		}

		auto rand_nd() noexcept -> double {
			return this->dist(this->engine);
		}
	};

	struct SensorOutput final {
		Matrix2Xd laserscan;
		Pose2d odometry;
	};

	inline auto sim_update(const SimulatorConstant& cons, SimulatorState& state, const Diff2wheelSpeed speed, const double dt) -> SensorOutput {
		const auto pose = state.pose;

		// update state ///////////////////////////////////////////////////////////////////////////
		auto speed_ = speed.to_pose2d_velocity(pose);
		speed_.xy *= 1 + state.rand_nd() * cons.noise_gain_ahead;
		speed_.th *= 1 + state.rand_nd() * cons.noise_gain_rotate;
		const auto new_pose = pose + speed_ * dt;

		// generate lidar data ////////////////////////////////////////////////////////////////////
		const auto map = cons.map.make_visible_lines(new_pose);
		for(i64 i = 0; i < cons.laser_count; ++i) {
			const double angle = cons.laser_angle_min + cons.laser_angle_range / cons.laser_count * i;
			const auto ray = Ray2d{Vector2d::Zero(), Vector2d{std::cos(angle), std::sin(angle)}};
			for(const auto& line : map) {
				
			}
		}

		return {};
	}
}

namespace ac_semi_2025::simulator {
	using impl::SimulatorConstant;
	using impl::SimulatorState;
	using impl::SensorOutput;
	using impl::sim_update;
}