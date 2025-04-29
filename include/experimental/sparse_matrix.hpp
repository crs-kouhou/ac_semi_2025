#pragma once

#include <vector>
#include <span>
#include <ranges>
#include <utility>
#include <algorithm>

#include "utility.hpp"

namespace ac_semi_2025::sparse_matrix {

	template<class E_ = void>
	struct Csr final {
		std::vector<E_> data;
		std::vector<usize> acc_row;
		std::vector<usize> col;

		auto row(const usize index) {
			auto f = [this](const usize i) -> std::pair<usize, E_&> {
				return {this->col[i], this->data[i]};
			};

			if(index >= this->acc_row.size() - 1) {
				return std::views::iota(usize(0), usize(0))
					| std::views::transform(std::move(f));
			}
			else {
				return std::views::iota(this->acc_row[index], this->acc_row[index + 1])
					| std::views::transform(std::move(f));
			}
		}

		auto c_row(const usize index) const {
			auto f = [this](const usize i) -> std::pair<usize, const E_&> {
				return {this->col[i], this->data[i]};
			};

			if(index >= this->acc_row.size() - 1) {
				return std::views::iota(usize(0), usize(0))
					| std::views::transform(std::move(f));
			}
			else {
				return std::views::iota(this->acc_row[index], this->acc_row[index + 1])
					| std::views::transform(std::move(f));
			}
		}

		auto view() {
			return std::views::iota(0u, this->acc_row.size() - 1)
				| std::views::transform([this](const usize i) {
					return this->row(i);
				});
		}

		auto c_view() const {
			return std::views::iota(0u, this->acc_row.size() - 1)
				| std::views::transform([this](const usize i) {
					return this->c_row(i);
				});
		}

		auto begin() {
			return this->view().begin();
		}

		auto end() {
			return this->view().end();
		}

		auto cbegin() const {
			return this->c_view().begin();
		}

		auto cend() const {
			return this->c_view().end();
		}
	};

	template<>
	struct Csr<> final {
		std::vector<usize> acc_row;
		std::vector<usize> col;

		auto row(const usize index) {
			auto f = [this](const usize i) -> usize {
				return this->col[i];
			};

			if(index >= this->acc_row.size() - 1) {
				return std::views::iota(usize(0), usize(0))
					| std::views::transform(std::move(f));
			}
			else {
				return std::views::iota(this->acc_row[index], this->acc_row[index + 1])
					| std::views::transform(std::move(f));
			}
		}

		auto c_row(const usize index) const {
			auto f = [this](const usize i) -> usize {
				return this->col[i];
			};

			if(index >= this->acc_row.size() - 1) {
				return std::views::iota(usize(0), usize(0))
					| std::views::transform(std::move(f));
			}
			else {
				return std::views::iota(this->acc_row[index], this->acc_row[index + 1])
					| std::views::transform(std::move(f));
			}
		}

		auto view() {
			return std::views::iota(0u, this->acc_row.size() - 1)
				| std::views::transform([this](const usize i) {
					return this->row(i);
				});
		}

		auto c_view() const {
			return std::views::iota(0u, this->acc_row.size() - 1)
				| std::views::transform([this](const usize i) {
					return this->c_row(i);
				});
		}

		auto begin() {
			return this->view().begin();
		}

		auto end() {
			return this->view().end();
		}

		auto cbegin() const {
			return this->c_view().begin();
		}

		auto cend() const {
			return this->c_view().end();
		}
	};

	template<class E_ = void>
	struct Coo final {
		std::vector<std::pair<std::pair<usize, usize>, E_>> buffer;
		bool is_sorted = false;

		static auto make(const usize buffer_size) {
			auto buffer = std::vector<std::pair<std::pair<usize, usize>, E_>>{};
			buffer.reserve(buffer_size);
			return Coo<E_> {
				.buffer = std::move(buffer),
				.is_sorted = false
			};
		}

		void add(const std::pair<usize, usize>& index, E_&& elem) {
			this->is_sorted = false;
			this->buffer.emplace_back(std::pair<std::pair<usize, usize>, E_>{index, std::move(elem)});
		}

		void sort() {
			std::ranges::sort(this->buffer);
			this->is_sorted = true;
		}

		auto to_csr(const usize row_size) && -> Csr<E_>
		// [[expect: indices are sorted by row and then by column]]
		{
			if(not this->is_sorted) {
				this->sort();
			}

			std::vector<E_> data{};
			std::vector<usize> acc_row{};
			std::vector<usize> col{};

			data.reserve(this->buffer.size());
			acc_row.reserve(row_size + 1);
			col.reserve(this->buffer.size());

			for(const auto& [index, elem] : this->buffer) {
				data.emplace_back(std::move(elem));
				col.emplace_back(index.second);
			}

			auto it = this->buffer.begin();
			for(usize i = 0; i < row_size; ++i) {
				acc_row.push_back(it - this->buffer.begin());
				while(it != this->buffer.end() && it->first.first == i) {
					++it;
				}
			}
			acc_row.push_back(this->buffer.size());

			return Csr<E_> {
				.data = std::move(data),
				.acc_row = std::move(acc_row),
				.col = std::move(col)
			};
		}
	};

	template<>
	struct Coo<> final {
		std::vector<std::pair<usize, usize>> buffer;
		bool is_sorted = false;

		static auto make(const usize buffer_size) {
			auto buffer = std::vector<std::pair<usize, usize>>{};
			buffer.reserve(buffer_size);
			return Coo<> {
				.buffer = std::move(buffer),
				.is_sorted = false
			};
		}

		void add(const std::pair<usize, usize>& index) {
			this->is_sorted = false;
			this->buffer.push_back(index);
		}

		void sort() {
			std::ranges::sort(this->buffer);
			this->is_sorted = true;
		}

		auto to_csr(const usize row_size) && -> Csr<> {
			if(not this->is_sorted) {
				this->sort();
			}

			std::vector<usize> acc_row{};
			std::vector<usize> col{};

			acc_row.reserve(row_size + 1);
			col.reserve(this->buffer.size());

			for(const auto& index : this->buffer) {
				col.push_back(index.second);
			}

			auto it = this->buffer.begin();
			for(usize i = 0; i < row_size; ++i) {
				acc_row.push_back(it - this->buffer.begin());
				while(it != this->buffer.end() && it->first == i) {
					++it;
				}
			}
			acc_row.push_back(this->buffer.size());

			return Csr<> {
				.acc_row = std::move(acc_row),
				.col = std::move(col)
			};
		}
	};
}
