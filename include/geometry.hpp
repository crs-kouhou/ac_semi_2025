#pragma once

#include <cmath>
#include <concepts>
#include <vector>
#include <utility>
#include <string>
#include <format>

#include <eigen3/Eigen/Dense>

namespace ac_semi_2025::geometry::impl {
	using Eigen::Vector2d;
	using Eigen::Isometry2d;

	template<class T_>
	concept edge = requires(const T_ e1, const T_ e2, const Vector2d p) {
		{intersect(e1, e2)} -> std::same_as<std::vector<Vector2d>>;
		{closest_p2e(p, e1)} -> std::same_as<std::pair<Vector2d, double>>;
	};

	// 姿勢
	struct Pose2d final {
		Vector2d xy;
		double th;

		constexpr auto to_isometry() const noexcept -> Isometry2d {
			auto ret = Isometry2d::Identity();
			ret.rotate(this->th).pretranslate(this->xy);
			return ret;
		}

		static constexpr auto from_isometry(const Isometry2d& isom) noexcept -> Pose2d {
			auto translation = isom.translation();
			const auto rotation = isom.rotation();
			return Pose2d{std::move(translation), std::atan2(rotation(1, 0), rotation(0, 0))};
		}

		constexpr auto to_str() const noexcept -> std::string {
			return std::format("xy : {{{}, {}}}, th : {}", this->xy(0), this->xy(1), this->th);
		}

		friend constexpr auto operator*(const Pose2d& l, const double r) noexcept -> Pose2d {
			return Pose2d{l.xy * r, l.th * r};
		}

		friend constexpr auto operator+(const Pose2d& l, const Pose2d& r) noexcept -> Pose2d {
			return Pose2d{l.xy + r.xy, l.th + r.th};
		}

		constexpr auto not_nan() const noexcept -> bool {
			return !std::isnan(this->xy(0)) && !std::isnan(this->xy(1)) && !std::isnan(this->th);
		}
	};

	// 端点2つによる線分表現
	struct Line2d final {
		Vector2d p1;
		Vector2d p2;
	};

	inline constexpr auto intersect(const Line2d& l, const Line2d& r) noexcept -> std::vector<Vector2d> {
		/// @todo 実装
		(void) l;
		(void) r;
		return {};
	}

	inline constexpr auto closest_p2e(const Vector2d& part, const Line2d& whole) noexcept -> std::pair<Vector2d, double> {
		const auto segment = whole.p2 - whole.p1;
		const auto from_p1 = part - whole.p1;
		const auto to_p2 = whole.p2 - part;

		if(from_p1.dot(segment) <= 0.0) {
			return std::pair{whole.p1, from_p1.norm()};
		}
		else if(to_p2.dot(segment) <= 0.0) {
			return std::pair{whole.p2, to_p2.norm()};
		}
		else {
			const auto point = from_p1.dot(segment) / segment.squaredNorm() * segment + whole.p1;
			return std::pair{point, (part - point).norm()};
		}
	}

	/// @brief 直線
	struct Ray2d final {
		Vector2d start;
		Vector2d direction;
	};

	/// @brief 折れ線
	struct Polyline2d final {
		std::vector<Vector2d> vertices;

		auto to_edges() const noexcept -> std::vector<Line2d> {
			const i64 m = this->vertices.size() - 1;
			std::vector<Line2d> edges(m);
			Vector2d last = this->vertices[0];
			for(i64 i = 1; i < m; ++i) {
				edges[i] = Line2d{last, this->vertices[i]};
				last = this->vertices[i];
			}

			return edges;
		}
	};

	/// @brief 閉じてる折れ線
	struct Polygon2d final {
		std::vector<Vector2d> vertices;

		auto to_edges() const noexcept -> std::vector<Line2d> {
			const i64 m = this->vertices.size();
			std::vector<Line2d> edges(m);
			Vector2d last = this->vertices[0];
			for(i64 i = 0; i < m - 1; ++i) {
				edges[i] = Line2d{last, this->vertices[i + 1]};
				last = this->vertices[i + 1];
			}
			edges[m - 1] = Line2d{last, this->vertices[0]};

			return edges;
		}
	};
}

namespace ac_semi_2025::geometry {
	using impl::edge;
	using impl::Line2d;
	using impl::Pose2d;
	using impl::Ray2d;
	using impl::Polyline2d;
	using impl::Polygon2d;
	using impl::closest_p2e;
}