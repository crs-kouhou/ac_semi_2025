#pragma once

#include <optional>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "utility.hpp"
#include "geometry.hpp"

namespace ac_semi_2025::carrot_pursuit::impl {
	using Eigen::Isometry2d;
	using Eigen::Vector2d;

	using geometry::Pose2d;

	struct CarrotPursuit final {
		/// @todo 実装。configurableな項目を入れておく
		int distance_threshold;
		int speed_determination_destination;

		auto update(const std::vector<Vector2d> &route, const Pose2d &current_pose, i64 &closest_ever_idx) const noexcept -> std::optional<Pose2d> {
			// 方向を求めるフェーズ
			auto dr = (route.at(closest_ever_idx) - current_pose.xy).eval();
			auto dr_1 = (route.at(closest_ever_idx + 1) - current_pose.xy).eval();
			while (dr.norm() > this->distance_threshold && dr_1.norm() < this->distance_threshold) {
				++closest_ever_idx;
				dr = route.at(closest_ever_idx) - current_pose.xy;
				dr_1 = route.at(closest_ever_idx + 1) - current_pose.xy;
			}
			// for(int n = closest_ever_idx;以下の条件;i++)みたいな気持ち
			// n個めでロボットからthis->distance_thresholdまでの距離の中にいて、
			// かつn+1個めでロボットからthis->distance_thresholdまでの距離の外にいるとき、n+1個めを目的地とする
			Vector2d speed = (dr_1 / dr_1.norm()).eval();
			// ベクトルの規格化

			// 速度を求めるフェーズ
			// 上手くいかなかったら消せばどこでも同じ速度で動くよ
			auto vector_traget = (route.at(closest_ever_idx + this->speed_determination_destination) - route.at(closest_ever_idx)).eval();
			// 現在地のノードからthis->speed_determination_destination個めのノードまでのベクトルを求める
			vector_traget.normalize();
			// ベクトルの規格化
			// speed *= vector_traget;
			// // 内積を求める

			// ワールド座標系からロボット座標系に変換するフェーズ
			Eigen::Rotation2Dd rotation(current_pose.th);
			speed = rotation * speed;

			return Pose2d {
				speed,
				0.0
			};
		}
	};
}

namespace ac_semi_2025::carrot_pursuit {
	using impl::CarrotPursuit;
}