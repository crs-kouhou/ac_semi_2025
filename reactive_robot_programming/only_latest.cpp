#include <thread>
#include <iostream>
#include <syncstream>
#include <random>
#include "valve_tap.hpp"

// using ac_semi_2025::valve_tap::impl::debug_print;

struct LaserScan {};
struct GlobalMap {};
struct PointCloud {};
struct Pose2d {};
struct Transform2d {};

struct RandomSleep final {
	std::mutex eng_mtx{};
	std::default_random_engine eng{std::random_device{}()};
	std::uniform_real_distribution<> dist{0.5, 3.0};

	void random_sleep() {
		const double du = [this] {
			std::unique_lock lck{this->eng_mtx};
			return this->dist(this->eng);
		}();
		std::this_thread::sleep_for(std::chrono::duration<double>{du});
	}
} rand_sleep{};


auto icp(PointCloud laser, Pose2d pose, GlobalMap map) noexcept -> std::tuple<Transform2d> {
	(void) laser;
	(void) pose;
	(void) map;

	std::osyncstream osy{std::cout};
	osy << "in icp." << std::endl;
	rand_sleep.random_sleep();

	Transform2d map2odom{};
	return map2odom;
}

auto odometry(Pose2d twist, Pose2d pose, Transform2d odom2base) noexcept -> std::tuple<Transform2d> {
	(void) twist;
	(void) pose;
	(void) odom2base;

	std::osyncstream osy{std::cout};
	osy << "in odometry." << std::endl;
	rand_sleep.random_sleep();
	
	Transform2d new_odom2base{};
	return new_odom2base;
}

auto calc_pose(Transform2d map2odom, Transform2d odom2base) noexcept -> std::tuple<Pose2d> {
	(void) map2odom;
	(void) odom2base;

	std::osyncstream osy{std::cout};
	osy << "in calc_pose." << std::endl;

	Pose2d pose{};
	return pose;
}

int main() {
	using ac_semi_2025::valve_tap::tap;
	using ac_semi_2025::valve_tap::valve;
	using ac_semi_2025::valve_tap::ValveExecutor;
	using ac_semi_2025::valve_tap::OpenedTap;

	auto [laser_tap, laser_send, laser_recv] = tap<PointCloud>();
	auto [twist_tap, twist_send, twist_recv] = tap<Pose2d>();
	auto [map2odom_tap, map2odom_send, map2odom_recv] = tap<Transform2d>();
	auto [odom2base_tap, odom2base_send, odom2base_recv] = tap<Transform2d>();
	auto [pose_tap, pose_send, pose_recv] = tap<Pose2d>();
	auto [map_tap, map_send, map_recv] = tap<GlobalMap>();

	std::stop_source ssource{};
	auto stoken = ssource.get_token();

	map_send.tap->set({});
	pose_send.tap->set({});
	odom2base_send.tap->set({});

	auto icp_valv = valve (
		std::forward_as_tuple(laser_recv)
		, std::forward_as_tuple(pose_recv, map_recv)
		, std::forward_as_tuple(std::move(map2odom_send))
		, icp
	);

	auto odometry_valv = valve (
		std::forward_as_tuple(twist_recv)
		, std::forward_as_tuple(pose_recv, odom2base_recv)
		, std::forward_as_tuple(std::move(odom2base_send))
		, odometry
	);

	auto calc_pose_valv = valve (
		std::forward_as_tuple(map2odom_recv, odom2base_recv)
		, std::forward_as_tuple()
		, std::forward_as_tuple(std::move(pose_send))
		, calc_pose
	);

	auto [executor, laser_opened, twist_opened, map_opened] = ValveExecutor::make (
		std::move(stoken)
		, std::vector {
			icp_valv.get()
			, odometry_valv.get()
			, calc_pose_valv.get()
		}
		, std::move(laser_send)
		, std::move(twist_send)
		, std::move(map_send)
	);
	std::jthread exec_thread{[&executor] {
		executor.run();
	}};

	std::jthread sigint_like{[&executor, &ssource, &laser_opened, &twist_opened, &map_opened] {
		while(true) {
			char c;
			std::cin >> c;
			if(c == 'q') {
				std::osyncstream osy{std::cout};
				osy << "exit." << std::endl;
				ssource.request_stop();
				executor.shutdown();
				return;
			}
			else {
				if(c == 'l') {
					laser_opened.pour_into({});
				}
				else if(c == 't') {
					twist_opened.pour_into({});
				}
				else if(c == 'm') {
					map_opened.pour_into({});
				}
			}
		}
	}};
}