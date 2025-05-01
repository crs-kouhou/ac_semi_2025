#pragma once

#include <charconv>
#include <vector>
#include <expected>
#include <string>
#include <string_view>
#include <utility>

#include <eigen3/Eigen/Dense>

#include "utility.hpp"
#include "geometry.hpp"
#include "flie_chainer.hpp"

namespace ac_semi_2025::read_edges::impl {
	using Eigen::Vector2d;

	using geometry::Line2d;
	using geometry::Polyline2d;
	using geometry::Polygon2d;
	using file_chainer::FileChainer;
	using file_chainer::FopenMode;

	template<class T_>
	inline auto parse(const std::string_view s) noexcept(false) -> T_ {
		T_ ret{};
		if(std::from_chars(s.begin(), s.end(), ret).ec != std::errc{}) {
			throw std::runtime_error{"parse: fail to parse."};
		}

		if(std::same_as<T_, double>) {
			ret /= 1000.0;
		}

		return ret;
	}

	inline auto read_until(FileChainer& fchainer, const char c) -> std::string_view {
		char buffer[1024]{};
		usize size{0};
		if(fchainer.read_until(buffer, sizeof(buffer), c, size).is_fail()) {
			throw std::runtime_error{*fchainer.err};
		}
		return std::string_view{buffer, size - 1};
	}

	inline auto readline(FileChainer& fchainer) -> std::string_view {
		return read_until(fchainer, '\n');
	}

	inline auto read_vector2d(FileChainer& fchainer) -> Vector2d {
		const auto x = parse<double>(read_until(fchainer, ' '));
		const auto y = parse<double>(read_until(fchainer, '\n'));
		return Vector2d{x, y};
	}

	inline auto read_polyline(FileChainer& fchainer) -> Polyline2d {
		const auto len = parse<i64>(readline(fchainer));
		if(len <= 1) {
			throw std::runtime_error{"read_edges: polyline is too short."};
		}
		Polyline2d ret{};
		ret.vertices.reserve(len);
		for(i64 i = 0; i < len; ++i) {
			ret.vertices.emplace_back(read_vector2d(fchainer));
		}

		return ret;
	}

	inline auto read_polygon(FileChainer& fchainer) -> Polygon2d {
		const auto len = parse<i64>(readline(fchainer));
		if(len <= 2) {
			throw std::runtime_error{"read_edges: polyline is too short."};
		}
		Polygon2d ret{};
		ret.vertices.reserve(len);
		for(i64 i = 0; i < len; ++i) {
			ret.vertices.emplace_back(read_vector2d(fchainer));
		}

		return ret;
	}

	inline auto read_route(const std::string_view filepath) -> std::expected<Polyline2d, std::string> {
		try {
			auto fchainer_ = FileChainer::make(std::string{filepath}.c_str(), FopenMode::r);
			if(!fchainer_) throw std::runtime_error{fchainer_.error()};
			auto fchainer = std::move(fchainer_.value());

			auto ret = read_polyline(fchainer);
			return std::expected<Polyline2d, std::string>{std::in_place, std::move(ret)};
		}
		catch(const std::runtime_error& e) {
			return std::expected<Polyline2d, std::string>{std::unexpect, e.what()};
		}
	}

	inline auto read_edges(const std::string_view filepath) -> std::expected<std::vector<Polygon2d>, std::string> {
		try {
			auto fchainer_ = FileChainer::make(std::string{filepath}.c_str(), FopenMode::r);
			if(!fchainer_) throw std::runtime_error{fchainer_.error()};
			auto fchainer = std::move(fchainer_.value());

			const auto n = parse<i64>(readline(fchainer));
			std::vector<Polygon2d> ret(n);
			for(auto& pline : ret) {
				pline = read_polygon(fchainer);
			}

			return std::expected<std::vector<Polygon2d>, std::string>{std::in_place, std::move(ret)};
		}
		catch(const std::runtime_error& e) {
			return std::expected<std::vector<Polygon2d>, std::string>{std::unexpect, e.what()};
		}
	}
}

namespace ac_semi_2025::read_edges {
	using impl::read_route;
	using impl::read_edges;
}
