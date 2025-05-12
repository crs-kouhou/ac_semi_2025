/**
 * @file utility.hpp
 * @author tamaki 21T
 * @brief 自己位置推定まわりで頻出の型や処理をまとめた。と言えば聞こえはいいが、事実上「その他.hpp」だ。
 * @version 0.1
 * @date 2025-04-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace ac_semi_2025::integer_type {
	using i64 = std::int64_t;
	using u64 = std::uint64_t;
	using i32 = std::int32_t;
	using u32 = std::uint32_t;
	using u8 = std::uint8_t;
	using usize = std::size_t;
}

namespace ac_semi_2025 {
	using namespace integer_type;
}