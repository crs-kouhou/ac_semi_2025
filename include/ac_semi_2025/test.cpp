#include <thread>
#include <atomic>
#include <iostream>
#include <vector>
#include <chrono>
#include <string_view>

#include "utility.hpp"
#include "geometry.hpp"
#include "sparse_matrix.hpp"
#include "icp_on_svd.hpp"
#include "read_edges.hpp"
// #include "global_map.hpp"
#include "ros_world.hpp"
#include "carrot_pursuit.hpp"

namespace test {
	using Eigen::Matrix2Xd;
	using Eigen::Vector2d;

	using namespace ac_semi_2025::integer_type;
	using namespace ac_semi_2025::geometry;
	using namespace ac_semi_2025::icp_on_svd;
	using namespace ac_semi_2025::read_edges;
	// using namespace ac_semi_2025::global_map;
	using namespace ac_semi_2025::ros_world;
	using namespace ac_semi_2025::carrot_pursuit;

	// ロボの定数と状態
	struct RobotConstant final {
		std::vector<Line2d> global_edges;
		Polyline2d route;
		CarrotPursuit carrot;
		i64 number_of_iteration;
	};
	struct RobotState final {
		Pose2d pose;
		i64 closest_milestone_index;
	};

	// ロボの更新式
	inline auto robot_update(const RobotConstant& cons, RobotState& state, const Matrix2Xd& laserscan, const double dt) noexcept(false) -> Pose2d {
		// read state /////////////////////////////////////////////////////////////////////////////
		const auto pose = state.pose;

		// ICP on SVD /////////////////////////////////////////////////////////////////////////////
		// const auto visible_edges = cons.map.make_visible_lines(pose);
		const auto g2l = pose.homogeneus_transform().inverse();
		auto edges = cons.global_edges;
		for(auto& edge : edges) {
			edge = Line2d{g2l * edge.p1, g2l * edge.p2};
		}
		const auto new_pose = icp_p2l(laserscan, edges, cons.number_of_iteration);

		// calc control input /////////////////////////////////////////////////////////////////////
		const auto speed = cons.carrot.update(cons.route.vertices, new_pose, state.closest_milestone_index);
		if(!speed.has_value()) {
			throw std::runtime_error{"Panic: cannot calc speed by carrot pursuit."};
		}

		// update state ///////////////////////////////////////////////////////////////////////////
		state.pose = new_pose + *speed * dt;

		return *speed;
	}

	struct MyClock final {
		std::chrono::time_point<std::chrono::steady_clock> last;

		static auto make() noexcept -> MyClock {
			return MyClock{std::chrono::steady_clock::now()};
		}

		auto watch() const noexcept -> std::chrono::duration<double> {
			const auto now = std::chrono::steady_clock::now();
			return now - this->last;
		}

		auto lap() noexcept -> std::chrono::duration<double> {
			const auto last = this->last;
			this->last = std::chrono::steady_clock::now();
			return this->last - last;
		}
	};

	void main() {
		using namespace std::string_view_literals;

		/// グローバルな図形情報を読み出し
		std::vector<Line2d> global_edges = [] {
			if(const auto res = read_edges("data/field.dat"sv)) {
				std::vector<Line2d> ret{};
				for(const auto& polygon : res.value()) {
					const auto lines = polygon.to_edges();
					ret.insert(ret.end(), lines.begin(), lines.end());
				}
				ret.shrink_to_fit();

				return ret;
			}
			else {
				throw std::runtime_error{res.error()};
			}
		}();

		/// ルート情報を読み出し
		Polyline2d route = [] {
			if(const auto res = read_route("data/test_route.dat"sv)) {
				return res.value();
			}
			else {
				throw std::runtime_error{res.error()};
			}
		}();

		// /// @todo グローバルマップを生成
		// const auto map = GlobalMap<Line2d>::from_shapes(shapes);

		// 終了用のキー受付
		std::atomic_bool stop_flag{true};
		std::jthread{[&stop_flag] {
			char dummy;
			std::cin >> dummy;
			stop_flag.store(false);
		}};

		// /// @todo 外界の初期化
		RosWorld node{};

		/// ロボットの初期化
		RobotConstant rb_cons {
			.global_edges = std::move(global_edges)
			, .route = std::move(route)
			, .carrot = CarrotPursuit {
				.distance_threshold = 5
				, .speed_determination_destination = 0
			}
			, .number_of_iteration = 50
		};
		RobotState rb_state {
			.pose = Pose2d{Vector2d::Zero(), 0.0}
			, .closest_milestone_index = 0
		};

		Pose2d control_input{Vector2d::Zero(), 0.0};

		auto sim_clock = MyClock::make();
		auto robo_clock = MyClock::make();
		// メインループ
		while(stop_flag.load()) {
			// calc world /////////////////////////////////////////////////////////////////////////
			const auto laserscan = node.update(control_input, sim_clock.lap().count());

			// calc robot /////////////////////////////////////////////////////////////////////////
			control_input = robot_update(rb_cons, rb_state, laserscan, robo_clock.lap().count());

			// snapshot ///////////////////////////////////////////////////////////////////////////
			// sim_state.snap(logger);
			// rb_state.snap(logger);
		}
	}
}

int main() {
	test::main();
}