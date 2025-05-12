/**
 * @file valve_tap.cpp
 * @author your name (you@domain.com)
 * @brief ROS--(仮)試作1。メモリをふんだんに動的確保している(が、おそらく工夫すれば静的にできるはず)
 * 欠落するとまずい類いの通信はまだ未対応。
 * @version 0.1
 * @date 2025-05-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <utility>
#include <tuple>
#include <memory>
#include <vector>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <stop_token>
#include <string>  // debug

#include "../ac_semi_2025/include/utility.hpp"

namespace ac_semi_2025::valve_tap::impl {
	// constなメンバ関数は全て並列に実行可能でなければならない
	template<class T_>
	concept datable = std::movable<T_>;

	inline void debug_print(const std::string& str) {
		std::osyncstream osy{std::cout};
		osy << str << std::endl;
	}

	struct Waker final {
		std::stop_token stoken;
		std::condition_variable cond_var{};
		std::mutex mtx{};
		bool waken{false};

		void awake() noexcept {
			std::unique_lock lck{this->mtx};
			this->waken = true;
			this->cond_var.notify_all();
		}

		void sleep() noexcept {
			std::unique_lock lck{this->mtx};
			this->cond_var.wait(lck, [this] {
				return this->stoken.stop_requested() || this->waken;
			});
			this->waken = false;
		}
	};

	struct ValveFuncBase {
		// おそらくキャプチャに参照しかないラムダ式のデストラクタはtrivialだと思うんだけど、
		// 処理系が参照以外のデータメンバを追加している可能性を排除出来なかったので、こうする
		virtual ~ValveFuncBase() = default;

		virtual void do_once(Waker&) noexcept = 0;
	};

	template<class F_>
	struct ValveFunc : ValveFuncBase {
		F_ f;

		constexpr ValveFunc(auto&& f) noexcept
		: ValveFuncBase{}
		, f{std::forward<decltype(f)>(f)}
		{}

		virtual void do_once(Waker& waker) noexcept override {
			this->f(waker);
		}
	};
	template<class F_>
	ValveFunc(F_&& f) -> ValveFunc<std::remove_cvref_t<decltype(f)>>;

	struct Valve final {
		std::unique_ptr<ValveFuncBase> func_up;
		std::atomic<u8> state;

		void pour_into() noexcept {
			this->state.fetch_or(0b10);
		}

		void mix_up(Waker& waker) noexcept {
			u8 expected = 0b10;
			if(this->state.compare_exchange_strong(expected, 0b01)) {
				this->func_up->do_once(waker);
				this->state.store(0b00);
			}
		}
	};

	template<datable Data_>
	struct Tap {
		std::vector<Valve *> dest_valves{};
		std::shared_ptr<Data_> data_sp{};
		std::mutex mtx{};

		void set(Data_&& data) noexcept {
			auto new_data_sp = std::make_shared<Data_>(std::move(data));
			std::unique_lock lck{this->mtx};
			this->data_sp = std::move(new_data_sp);

			for(Valve *const valv_p : dest_valves) {
				valv_p->pour_into();
			}
		}

		auto get_shared() noexcept -> std::shared_ptr<Data_> {
			std::unique_lock lck{this->mtx};
			if(!this->data_sp) throw std::runtime_error{"null!!!!"};
			return this->data_sp;
		}

		void add_dest_valve(Valve *const valv_p) noexcept {
			this->dest_valves.push_back(valv_p);
		}
	};

	template<datable Data_>
	struct Inlet final {
		Tap<Data_> * tap;
	};

	template<datable Data_>
	struct Outlet final {
		Tap<Data_> * tap;
	};

	template<datable Data_>
	inline auto tap() -> std::tuple<std::unique_ptr<Tap<Data_>>, Inlet<Data_>, Outlet<Data_>> {
		auto tap = std::make_unique<Tap<Data_>>();
		const auto tap_p = tap.get();
		return {std::move(tap), Inlet<Data_>{tap_p}, Outlet<Data_>{tap_p}};
	}

	template<datable ... TriggerDatas_, datable ... NonTriggerDatas_, datable ... DestDatas_>
	inline auto valve_func_inner (
		std::tuple<Outlet<TriggerDatas_>& ...>&& trigger
		, std::tuple<Outlet<NonTriggerDatas_>& ...>&& non_trigger
		, std::tuple<Inlet<DestDatas_>&& ...>&& dest
		, auto&& f
	) noexcept {
		return [&trigger, &non_trigger, &dest, f] <
			usize ... triggers_
			, usize ... non_triggers_
			, usize ... dests_
		> (
			std::index_sequence<triggers_ ...>
			, std::index_sequence<non_triggers_ ...>
			, std::index_sequence<dests_ ...>
		) mutable {
			std::array<void *, sizeof...(TriggerDatas_) + sizeof...(NonTriggerDatas_) + sizeof...(DestDatas_)> taps{};
			
			((taps[triggers_] = static_cast<void *>(std::get<triggers_>(trigger).tap)), ...);
			((taps[sizeof...(triggers_) + non_triggers_] = static_cast<void *>(std::get<non_triggers_>(non_trigger).tap)), ...);
			((taps[sizeof...(triggers_) + sizeof...(non_triggers_) + dests_] = static_cast<void *>(std::get<dests_>(dest).tap)), ...);

			return [taps, f](Waker& waker) mutable noexcept -> void {
				auto ret = f (
					*static_cast<Tap<TriggerDatas_> *>(taps[triggers_])->get_shared() ...
					, *static_cast<Tap<NonTriggerDatas_> *>(taps[sizeof...(triggers_) + non_triggers_])->get_shared() ...
				);

				(static_cast<Tap<DestDatas_> *> (
					taps[sizeof...(triggers_) + sizeof...(non_triggers_) + dests_])->set(std::move(std::get<dests_>(ret))
				), ...);

				waker.awake();
			};
		} (
			std::index_sequence_for<TriggerDatas_ ...>{}
			, std::index_sequence_for<NonTriggerDatas_ ...>{}
			, std::index_sequence_for<DestDatas_ ...>{}
		);
	}

	// f(...)は並列に実行されても問題ない関数呼び出しでなければならない
	template<datable ... TriggerDatas_, datable ... NonTriggerDatas_, datable ... DestDatas_>
	inline auto valve (
		std::tuple<Outlet<TriggerDatas_>& ...>&& trigger
		, std::tuple<Outlet<NonTriggerDatas_>& ...>&& non_trigger
		, std::tuple<Inlet<DestDatas_>&& ...>&& dest
		, auto&& f
	) noexcept -> std::unique_ptr<Valve> requires
		requires (
			const TriggerDatas_& ... trigger_datas
			, const NonTriggerDatas_& ... non_trigger_datas
		) {
			{f(trigger_datas ..., non_trigger_datas ...)} noexcept -> std::same_as<std::tuple<DestDatas_ ...>>;
		}
	{
		auto valv_func = ValveFunc {
			valve_func_inner(std::move(trigger), std::move(non_trigger), std::move(dest), std::move(f))
		};
		auto valv_up = std::make_unique<Valve>(std::make_unique<decltype(valv_func)>(std::move(valv_func)));
		
		[&trigger, valv_p = valv_up.get()]<usize ... triggers_>(std::index_sequence<triggers_ ...>) {
			(std::get<triggers_>(trigger).tap->add_dest_valve(valv_p), ...);
		}(std::index_sequence_for<TriggerDatas_ ...>{});

		return valv_up;
	}

	template<datable Data_>
	struct OpenedTap final {
		Inlet<Data_> in;
		Waker * waker_p;

		static auto make(Inlet<Data_>&& in, Waker& waker) -> OpenedTap {
			return OpenedTap{std::move(in), &waker};
		}

		void pour_into(Data_&& data) noexcept {
			this->in.tap->set(std::move(data));
			this->waker_p->awake();
		}
	};

	struct ValveExecutor final {
		std::vector<Valve *> valves;
		std::unique_ptr<Waker> waker;
		std::stop_token stoken;

		template<datable ... OpenedDatas_>
		static auto make(std::stop_token&& stoken, std::vector<Valve *>&& valves, Inlet<OpenedDatas_>&& ... opened) noexcept -> std::tuple<ValveExecutor, OpenedTap<OpenedDatas_> ...> {
			auto executor = ValveExecutor {
				std::move(valves)
				, std::make_unique<Waker>(stoken)
				, std::move(stoken)
			};

			const auto waker_p = executor.waker.get();

			return {
				std::move(executor)
				, OpenedTap<OpenedDatas_>::make(std::move(opened), *waker_p) ...
			};
		}

		void run() noexcept {
			while(!this->stoken.stop_requested()) {
				debug_print("looping...");
				this->waker->sleep();

				for(Valve * valve : this->valves) {
					valve->mix_up(*this->waker);
				}
			}
		}

		void shutdown() noexcept {
			this->waker->awake();
		}
	};
}

namespace ac_semi_2025::valve_tap {
	using impl::datable;
	using impl::Valve;
	using impl::valve;
	using impl::Tap;
	using impl::Inlet;
	using impl::Outlet;
	using impl::tap;
	using impl::OpenedTap;
	using impl::ValveExecutor;
}