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

	struct PID {
		double K_P = 0.0;
		double K_I = 0.0;
		double K_D = 0.0;
		double last_P = 0.0;
		double last_integrated_P = 0.0;
		double feedforward = 0.0;


		void make(double K_P, double K_I, double K_D) noexcept {
			this->K_P = K_P;
			this->K_I = K_I;
			this->K_D = K_D;
		}
		void make(double K_P, double K_I, double K_D,double feedforward) noexcept {
			this->K_P = K_P;
			this->K_I = K_I;
			this->K_D = K_D;
			this->feedforward = feedforward;
		}
		auto update(double P,double dt) noexcept -> double {
			auto dP = (P - this->last_P) / dt;
			auto integrated_P = this->last_integrated_P + P * dt;
			auto ret = this->K_P * P + this->K_I * integrated_P + this->K_D * dP + this->feedforward;
			this->last_P = P;
			this->last_integrated_P = integrated_P;
			return ret;
		}

		
	};

	

	struct CarrotPursuit final {
		/// @todo 実装。configurableな項目を入れておく
		// int distance_threshold;
		// int speed_determination_destination;
		PID PID_x;
		PID PID_y;

		auto update(const std::vector<Vector2d> &route, const Pose2d &current_pose, i64 &closest_ever_idx , const double dt) noexcept -> std::optional<Pose2d> {
			// 方向を求めるフェーズ
			auto road_vector = (route[closest_ever_idx + 1] - route[closest_ever_idx]).eval();
			auto distination_vector = (route[closest_ever_idx + 1] - current_pose.xy).eval();
			while (road_vector.dot(distination_vector) < 0){
				closest_ever_idx++;
				if (closest_ever_idx >= route.size() - 1) {
					return std::nullopt;
				}
				road_vector = (route[closest_ever_idx + 1] - route[closest_ever_idx]).eval();
				distination_vector = (route[closest_ever_idx + 1] - current_pose.xy).eval();
			}
			Eigen::Rotation2Dd _90degree_rotation(-M_PI / 2);
			Eigen::Rotation2Dd degree90_rotation(M_PI / 2);
			auto P_x = road_vector.dot(distination_vector) / road_vector.norm();
			auto P_y = (road_vector.dot(distination_vector * _90degree_rotation.toRotationMatrix())) / road_vector.norm();
			auto distination_vector_target =  PID_x.update(P_x, dt) * road_vector.normalized() + PID_y.update(P_y, dt) * degree90_rotation.toRotationMatrix()* road_vector.normalized();
			
			return Pose2d{distination_vector_target,0.0};
			
		}
	};
}

namespace ac_semi_2025::carrot_pursuit {
	using impl::CarrotPursuit;
}