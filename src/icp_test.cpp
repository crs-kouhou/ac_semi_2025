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
using namespace ac_semi_2025::icp_on_svd;

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
	for(i64 i = 0; i < 1000; ++i) icp_p2l(points, l, 50);
	const auto end = std::chrono::high_resolution_clock::now();
	std::println(std::cerr, "end: {}", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
}