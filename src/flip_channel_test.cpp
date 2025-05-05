#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "../include/flip_channel.hpp"

using namespace std::chrono_literals;
using namespace ac_semi_2025;
using flip_channel::channel;

int main() {
	auto [str_send, str_recv] = channel<std::string>();

	// Sender and Receiver cannot copy.
	// auto send_copy = str_send;  error!
	// auto recv_copy = str_recv;  error!

	// you must move them.
	std::jthread sending {[str = std::move(str_send)] mutable {
		while(true) {
			std::string s;
			std::cin >> s;
			str.send(std::string{s});
			if(s[0] == 'q') return;
		}
	}};

	std::jthread recving {[str = std::move(str_recv)] mutable {
		while(true) {
			while(const auto s_ = str.try_recv(500ms)) {
				const auto& s = *s_;
				if(s[0] == 'q') {
					return;
				}
				else {
					std::cout << s << std::endl;
				}
			}
		}
	}};
}