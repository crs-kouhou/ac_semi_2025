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

	inline auto clamp(const double x, const double min, const double max) -> double {
		return std::max(std::min(x, max), min);
	}

	struct FeedForwardedPid final {
		struct Constant final {
			double kp;
			double ki;
			double kd;
			double i_max;
			double feed_forward;
		};

		const Constant cons;
		double last_p{0.0};
		double last_i{0.0};

		static auto make(const Constant& cons) noexcept -> FeedForwardedPid {
			return FeedForwardedPid{cons};
		}

		auto update(const double p, const double dt) noexcept -> double {
			const auto d = (p - this->last_p) / dt;
			const auto i = clamp(this->last_i + p * dt, -this->cons.i_max, this->cons.i_max);
			const auto ret = this->cons.kp * p + this->cons.ki * i + this->cons.kd * d + this->cons.feed_forward;
			this->last_p = p;
			this->last_i = i;
			return ret;
		}
	};

	// 現在位置と目標位置との差分を、ルート上の目標位置近傍での局所座標系で表し、
	// 成分ごとにFFつきPID制御器に掛け、これを制御入力とする。
	// なお、機体が戻る方向に動かないよう、目標位置の更新は工夫している
	//
	// 以降、局所座標系の接線方向をs, 法線方向をtとする
	struct CarrotPursuit final {
		FeedForwardedPid controller_s;
		FeedForwardedPid controller_t;
		i64 target_idx{1};

		static auto make(const FeedForwardedPid::Constant& s, const FeedForwardedPid::Constant& t) noexcept -> CarrotPursuit {
			return CarrotPursuit {
				.controller_s = FeedForwardedPid{s}
				, .controller_t = FeedForwardedPid{t}
			};
		}

		auto update(const std::vector<Vector2d> &route, const Pose2d &current_pose, const double dt) noexcept -> std::optional<Pose2d> {
			Vector2d unnormalized_s = (route[this->target_idx] - route[this->target_idx - 1]).eval();
			Vector2d to_dest = (route[this->target_idx] - current_pose.xy).eval();
			// this->target_idxは減ることはない。よって機体が軌道上を逆戻りすることはない
			// 目標位置の接線ベクトルと現在位置と目標位置の差分ベクトルの内積の正負により、
			// 目標位置を通過したかを判定できる
			while(unnormalized_s.dot(to_dest) < 0) {
				++this->target_idx;
				if (i64(route.size()) <= this->target_idx) {  // ゴールしたら
					// その場で停止(@todo: ホントはゴールへ近づき、ゴールとの距離や速度が落ち着いてから停止すべき)
					return Pose2d{Vector2d::Zero(), 0.0};
				}
				unnormalized_s = (route[this->target_idx] - route[this->target_idx - 1]).eval();
				to_dest = (route[this->target_idx] - current_pose.xy).eval();
			}

			// 目標位置近傍での局所座標系で成分表示
			const auto s = unnormalized_s.normalized();
			const auto t = Vector2d{-s(1), s(0)};
			const auto p_s = s.dot(to_dest);
			const auto p_t = t.dot(to_dest);

			// 各成分を制御器にかけ、その出力を制御入力の機体速度とする
			const double out_s = this->controller_s.update(p_s, dt);
			const double out_t = this->controller_t.update(p_t, dt);
			auto target_speed = out_s * s + out_t * t;
			
			return Pose2d{target_speed, 0.0};
			
		}
	};
}

namespace ac_semi_2025::carrot_pursuit {
	using impl::FeedForwardedPid;
	using impl::CarrotPursuit;
}