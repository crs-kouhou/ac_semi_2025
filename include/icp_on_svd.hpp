/**
 * @file icp_p2l.hpp
 * @author tsubasa 23S, tamaki 21T
 * @brief Iterative Closest Point(ICP)による2D LiDAR + オドメトリを用いた自己位置推定コード。マップ情報を線分で持っているので軽量高速。
 * @version 0.1
 * @date 2025-04-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <cmath>
#include <utility>
#include <vector>
#include <limits>

#include <eigen3/Eigen/Dense>

#include "utility.hpp"
#include "geometry.hpp"

namespace ac_semi_2025::icp_on_svd::impl {
	using Eigen::Matrix2Xd;
	using Eigen::Matrix2d;
	using Eigen::Vector2d;
	using Eigen::Isometry2d;
	using Eigen::JacobiSVD;

	using geometry::edge;
	using geometry::Line2d;
	using geometry::Pose2d;
	using geometry::closest_e2e;
	using geometry::closest_p2e;

	/// @brief 点群を線分群にfittingする ICP on SVD
	/// 線分数は点数に比べ十分少ないとする
	inline auto icp_p2l(const Matrix2Xd& from, std::vector<Line2d> to, const i64 number_of_iteration) noexcept -> Pose2d {
		// fromを、重心を原点とする座標系に変換したものを用意
		const auto [from_, from_mean] = [&from] {
			auto from_ = from;
			const auto from_mean = from.rowwise().mean();
			static_assert(decltype(from_mean)::RowsAtCompileTime == 2);
			from_.colwise() -= from_mean;
			return std::pair{std::move(from_), std::move(from_mean)};
		}();

		// fromをclosest_pointsに合わせる変換を計算し、その変換を合成、toに適用していく
		auto closest_points = Matrix2Xd{2, from.cols()};
		auto total_transform = Isometry2d::Identity();
		for(i64 iloop = 0; iloop < number_of_iteration; iloop++) {
			// fromの各点の最近接点を求める
			for(i64 ip = 0; ip < i64(from.cols()); ++ip) {
				Vector2d closest_point{0.0, 0.0};
				double closest_distance = std::numeric_limits<double>::infinity();
				for (i64 iq = 0; iq < i64(to.size()); iq++) {
					const auto [point, distance] = closest_p2e(from.col(ip), to[iq]);
					if(distance < closest_distance) {
						closest_distance = distance;
						closest_point = point;
					}
				}

				closest_points.col(ip) = closest_point;
			}

			// closest_pointsを、重心を原点とする座標系に直す
			const auto closest_points_mean = closest_points.rowwise().mean().eval();
			static_assert(decltype(closest_points_mean)::RowsAtCompileTime == 2);
			closest_points.colwise() -= closest_points_mean;

			// SVDで最適な剛体変換を求める
			const auto cross_covariance = (from_) * closest_points.transpose();
			static_assert(decltype(cross_covariance)::RowsAtCompileTime == 2 && decltype(cross_covariance)::ColsAtCompileTime == 2);

			// SVD分解
			JacobiSVD<Matrix2d> svd(cross_covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
			const auto u = svd.matrixU();
			const auto v = svd.matrixV();

			// 最適な回転移動を計算(if文内処理の詳細はjres.124.028.pdfなどを参照)
			auto optimized_rotation = (v * u.transpose()).eval();
			if(optimized_rotation.determinant() < 0.0) {
				optimized_rotation(1, 1) = -optimized_rotation(1, 1);
			}

			// 最適な平行移動を計算
			auto optimized_translation = closest_points_mean - optimized_rotation * from_mean;
			static_assert(decltype(optimized_translation)::RowsAtCompileTime == 2 && decltype(optimized_translation)::ColsAtCompileTime == 1);

			// 最適な剛体変換に合わせ、それを蓄積する
			auto optimized_transform = Isometry2d::Identity();
			optimized_transform.rotate(optimized_rotation).pretranslate(optimized_translation);

			total_transform = optimized_transform * total_transform;

			// 線分数が十分に小さいため計算が少なくて済むよう、toのほうを動かしてやる。適用すべきは逆変換である事に注意
			const auto inv_transform = optimized_transform.inverse();
			for(auto& toi : to) {
				toi.p1 = inv_transform * toi.p1;
				toi.p2 = inv_transform * toi.p2;
			}
		}

		// Pose2Dにして返す
		const auto translation = total_transform.translation();
		const auto rotation = total_transform.rotation();
		return Pose2d{translation, std::atan2(rotation(1, 0), rotation(0, 0))};
	}

	template<edge Edge_>
	inline auto icp_e2e(const std::vector<Edge_>& from, const std::vector<Edge_>& to, const i64 number_of_iteration) noexcept -> Pose2d {
		/// @todo 実装
		(void) from;
		(void) to;
		(void) number_of_iteration;
		return {};
	}
}

namespace ac_semi_2025::icp_on_svd {
	using impl::icp_p2l;
	using impl::icp_e2e;
}