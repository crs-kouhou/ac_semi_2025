#pragma once

#include <cmath>

#include <vector>
#include <tuple>
#include <compare>

#include <eigen3/Eigen/Dense>

#include "utility.hpp"
#include "geometry.hpp"
#include "sparse_matrix.hpp"

namespace ac_semi_2025::global_map::impl {
	using Eigen::Vector2d;
	
	using geometry::edge;
	using geometry::Line2d;
	using geometry::Pose2d;
	using sparse_matrix::Csr;

	/// マップの各曲線は重複せず、またそれぞれ曲線は交点を端点以外に持たないようにしておく
	template<edge Edge_>
	struct GlobalMap final {
		Csr<Edge_> edges;
		std::vector<Vector2d> vertices;

		/// @brief 曲線群からGlobalMapを生成
		static auto from_shapes(const auto&) noexcept -> GlobalMap {
			
		}

		/// @todo 根本から見直す必要あり。
		/// thetaでソート後、一つずつ頂点を舐める必要がある
		/// dr/dthetaが発散する点で分割する必要がある
		/// 図形処理の流れとしては、
		/// --ユーザー入力--> 基本図形 --ロボットが動く前--> 交点列挙
		/// --毎ループ(ローカル極座標変換、dtheta/dr==0で分割、thetaでスイープ)--> ローカル直交座標での各辺
		/// 
		/// @brief グローバル座標系でのマップから、ローカル座標系での可視なマップを計算
		/// make_visible_linesの計算量は、端点数をNとして O(NlogN)。ソート分となる
		/// @todo 一部でも見える曲線は全て載せてしまっている。手前の曲線の端点を通る光線で、奥の曲線をぶった切るべき
		auto make_visible_lines(const Pose2d& pose) const noexcept -> std::vector<Edge_> {
			const i64 n = this->positions.size();
			const auto global2local = pose.to_isometry().inverse();

			// マップ曲線をローカル極座標系に持ってくる
			struct RThetaVertex final {
				double r2;
				double th;
				i64 idx;

				constexpr auto operator<=>(const RThetaVertex&) const noexcept = default;
				constexpr auto operator==(const RThetaVertex&) const noexcept -> bool = default;
			};
			std::vector<RThetaVertex> local_vertices{};
			local_vertices.reserve(n);
			for(i64 i = 0; i < n; ++i) {
				const Vector2d pos = global2local * this->positions[i];
				const double r2 = pos.squaredNorm();
				const double th = std::atan2(pos(1), pos(0));
				local_vertices.emplace_back(r2, th, i);
			}
			
			// theta, r2, idxの順の辞書式ソート
			std::ranges::sort(local_vertices, [](const RThetaVertex& l, const RThetaVertex& r) -> bool {
				return l.th != r.th ? l.th < r.th :
					l.r2 != r.r2 ? l.r2 < r.r2 :
					l.idx < r.idx
				;
			});

			// idxから頂点を取得するための配列
			std::vector<i64> inv_indices(n);
			for(i64 i = 0; i < n; ++i) {
				inv_indices[local_vertices[i].idx] = i;
			}
			const auto get = [&local_vertices, &inv_indices](const i64 idx) noexcept -> RThetaVertex {
				return local_vertices[inv_indices[idx]];
			};

			std::vector<Edge_> ret{};
			ret.reserve(n);

			// thの小さいほうから、可視な曲線を見つけていく
			i64 leftmost_idx = 0;
			while(leftmost_idx < n) {
				/// @todo leftmost_idxから一つだけthetaの負のほうへ戻る必要がある
				/// 負のほうに繋がっている中で、最もr2が小さいものから始めていかなければならない
				i64 last_idx = leftmost_idx;
				auto nexts = this->edges.c_row(last_idx);

				// DAGの隣接頂点の中で最も原点に近いものを次々選んでいく
				while(nexts.size() > 1) {
					std::optional<std::pair<usize, const Edge_&>> next{std::nullopt};
					for(const auto& [idx, edge] : nexts) {
					// 一つ前やthetaが減る向きに戻ろうとしないよう注意
					if(i64(idx) == last_idx || get(idx).th < get(last_idx).th) continue;
						const auto& [next_idx, _] = *next;
						if(!next.has_value() || get(idx).r2 < get(next_idx).r2) {
							next.emplace(idx, edge);
						}
					}
					if(!next.has_value()) break;

					const auto [next_idx, edge] = *next;
					ret.emplace_back(edge);
					last_idx = next_idx;
					nexts = this->edges.c_row(next_idx);
				}

				leftmost_idx = last_idx + 1;
			}

			// ローカル座標系に変換
			/// @todo 実装

			return ret;
		}
	};
}

namespace ac_semi_2025::global_map {
	using impl::GlobalMap;
}