#pragma once

#include <utility>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <memory>
#include <optional>

namespace ac_semi_2025::flip_channel::impl {
	using std::chrono::duration;

	template<class T_>
	struct Channel final {
		std::optional<T_> buffer{};
		std::mutex mtx{};
		std::condition_variable cond_var{};

		void set(T_&& v) {
			std::unique_lock lck{this->mtx};
			this->buffer = std::move(v);
			this->cond_var.notify_all();
		}

		template<class Rep_, class Period_>
		auto try_get(const duration<Rep_, Period_>& rel_time) -> std::optional<T_> {
			std::unique_lock lck{this->mtx};
			if(this->cond_var.wait_for(lck, rel_time, [this]() -> bool {
				return this->buffer.has_value();
			})) {
				auto ret = std::move(this->buffer);
				this->buffer.reset();
				return ret;
			}
			else {
				return std::nullopt;
			}
		}
	};

	template<class T_>
	struct Sender final {
		std::shared_ptr<Channel<T_>> channel;

		Sender() = delete;
		Sender(const Sender&) = delete;
		auto operator=(const Sender&) -> Sender& = delete;

		Sender(Sender&&) = default;
		auto operator=(Sender&&) -> Sender& = default;
		~Sender() = default;

		Sender(const std::shared_ptr<Channel<T_>>& channel)
			: channel{channel}
		{}

		void send(T_&& v) {
			this->channel->set(std::move(v));
		}
	};

	template<class T_>
	struct Receiver final {
		std::shared_ptr<Channel<T_>> channel;

		Receiver() = delete;
		Receiver(const Receiver&) = delete;
		auto operator=(const Receiver&) -> Receiver& = delete;

		Receiver(Receiver&&) = default;
		auto operator=(Receiver&&) -> Receiver& = default;
		~Receiver() = default;

		Receiver(const std::shared_ptr<Channel<T_>>& channel)
			: channel{channel}
		{}

		template<class Rep_, class Period_>
		auto try_recv(const duration<Rep_, Period_>& rel_time) -> std::optional<T_> {
			return this->channel->try_get(rel_time);
		}
	};

	template<class T_>
	inline auto channel() -> std::pair<Sender<T_>, Receiver<T_>> {
		auto channel_sp = std::make_shared<Channel<T_>>();
		auto sender = Sender<T_>{channel_sp};
		auto receiver = Receiver<T_>{channel_sp};
		return {std::move(sender), std::move(receiver)};
	}
}

namespace ac_semi_2025::flip_channel {
	using impl::Channel;
	using impl::Sender;
	using impl::Receiver;
	using impl::channel;
}