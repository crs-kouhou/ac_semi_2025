#include <random>
#include <numbers>
#include <limits>
#include <print>
#include <iostream>
#include <chrono>

#include <eigen3/Eigen/Dense>

#include "../include/ac_semi_2025/read_edges.hpp"
#include "../include/ac_semi_2025/icp_on_svd.hpp"

using Eigen::Vector2d;
using Eigen::Matrix2d;
using Eigen::Matrix2Xd;
using Eigen::Isometry2d;
using Eigen::JacobiSVD;

using namespace ac_semi_2025::integer_type;
using namespace ac_semi_2025::geometry;
using namespace ac_semi_2025::read_edges;

/// @brief 点群を線分群にfittingする ICP on SVD
/// 線分数は点数に比べ十分少ないとする
inline auto icp_p2l(const Matrix2Xd& from, std::vector<Line2d> to, const i64 number_of_iteration) noexcept -> Pose2d {
	// // 出力
	// std::println("{}", number_of_iteration);

	// fromを、重心を原点とする座標系に変換したものを用意
	auto from_ = from;
	const auto from_mean = from.rowwise().mean();
	static_assert(decltype(from_mean)::RowsAtCompileTime == 2);
	from_.colwise() -= from_mean;  // これ以降from_は変更されない

	// fromをclosest_pointsに合わせる変換を計算し、その変換を合成、toに適用していく
	auto closest_points = Matrix2Xd{2, from.cols()};
	auto total_transform = Isometry2d::Identity();
	for(i64 iloop = 0; iloop < number_of_iteration; iloop++) {
		// std::println("{}", to.size());
		// for(const auto& toi : to) {
		// 	std::println("{} {} {} {}", toi.p1(0), toi.p2(0), toi.p1(1), toi.p2(1));
		// }

		// fromの各点の最近接点を求める
		for(i64 ip = 0; ip < i64(from.cols()); ++ip) {
			Vector2d closest_point{};
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

		// std::println("{}", closest_points.cols());
		// for(i64 i = 0; i < closest_points.cols(); ++i) {
		// 	std::println("{} {}", closest_points.col(i)(0), closest_points.col(i)(1));
		// }

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
		// std::println(std::cerr, "translate: {} {}", optimized_translation(0), optimized_translation(1));
		// std::println(std::cerr, "from_mean: {} {}", from_mean(0), from_mean(1));
		// std::println(std::cerr, "closest_points_mean: {} {}", closest_points_mean(0), closest_points_mean(1));

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

int main() {
	using namespace std::string_view_literals;
	using std::numbers::pi;

	std::default_random_engine eng{};
	std::uniform_real_distribution<double> rect_x{-3.0, 3.0};
	std::uniform_real_distribution<double> rect_y{-1.0, 1.0};
	const auto gen_vec2d = [&rect_x, &rect_y](std::default_random_engine& eng) -> Vector2d {
		return Vector2d{rect_x(eng), rect_y(eng)};
	};

	constexpr auto intersect = [](const Line2d& l, const double th) -> double {
		Matrix2d a{};
		a.col(0) = Vector2d{std::cos(th), std::sin(th)};
		a.col(1) = l.p1 - l.p2;
		const Vector2d ts = a.inverse() * l.p1;
		if(ts(0) >= 0 && 0 <= ts(1) && ts(1) <= 1) {
			return ts(0);
		}
		else return std::numeric_limits<double>::infinity();
	};

	const i64 m = 20;
	std::vector<Line2d> l(m);
	l[0] = Line2d {
		Vector2d{-3.1, -1.1}
		, Vector2d{3.1, -1.1}
	};
	l[1] = Line2d {
		Vector2d{3.1, -1.1}
		, Vector2d{3.1, 1.1}
	};
	l[2] = Line2d {
		Vector2d{3.1, 1.1}
		, Vector2d{-3.1, 1.1}
	};
	l[3] = Line2d {
		Vector2d{-3.1, 1.1}
		, Vector2d{-3.1, -1.1}
	};
	// l[4] = Line2d {
	// 	Vector2d{-1.0, 0.5}
	// 	, Vector2d{1.0, 0.5}
	// };
	for(i64 i = 4; i < m; ++i) {
		l[i] = Line2d{gen_vec2d(eng), gen_vec2d(eng)};
	}

	const i64 n = 2000;
	Matrix2Xd p{2, n};
	for(i64 i = 0; i < n; ++i) {
		const double th = 2 * pi / n * i;
		double r = std::numeric_limits<double>::infinity();
		for(const Line2d& li : l) {
			const double r_ = intersect(li, th);
			if(std::isfinite(r_) && r_ < r) {
				r = r_;
			}
		}
		if(!std::isfinite(r)) throw std::runtime_error{"laser range contains inf."};
		p.col(i) = Vector2d{std::cos(th), std::sin(th)} * r;
	}

	// std::println("{}", n);
	// for(i64 i = 0; i < n; ++i) {
	// 	const auto pi = p.col(i);
	// 	std::println("{} {}", pi(0), pi(1));
	// }
	// std::println("{}", 1);
	// std::println("{}", m);
	// for(const auto& li : l) {
	// 	std::println("{} {} {} {}", li.p1(0), li.p2(0), li.p1(1), li.p2(1));
	// }
	
	std::uniform_real_distribution<> err_th{0.0, 2 * pi};
	const auto err_trans = Pose2d {
		Vector2d{std::cos(err_th(eng)), std::sin(err_th(eng))} * 0.5
		, err_th(eng) / 360.0 * 30
	}.homogeneous_transform();

	const auto points = err_trans * p;
	static_assert(decltype(points)::RowsAtCompileTime == 2);
	// std::println("{}", n);
	// for(i64 i = 0; i < n; ++i) {
	// 	const auto pi = points.col(i);
	// 	std::println("{} {}", pi(0), pi(1));
	// }
	const auto start = std::chrono::high_resolution_clock::now();
	icp_p2l(points, l, 50);
	const auto end = std::chrono::high_resolution_clock::now();
	std::println(std::cerr, "end: {}", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
}