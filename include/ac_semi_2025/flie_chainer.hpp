#pragma once

#include <cstdio>
#include <utility>
#include <string>
#include <optional>
#include <expected>
#include "utility.hpp"

namespace ac_semi_2025::file_chainer {
	enum class SeekOrigin: int {
		set = SEEK_SET
		, cur = SEEK_CUR
		, end = SEEK_END
	};

	enum class FopenMode: u8 {
		r
		, w
		, a
		, rp
		, wp
		, ap
		, rb
		, wb
		, ab
		, rbp
		, wbp
		, abp
	};

	// モナディックな操作ができるように(エラーが出たらすぐやめられるように)
	struct FileChainer final {
		std::FILE * file{nullptr};
		std::optional<std::string> err{std::nullopt};

		FileChainer(const FileChainer&) = delete;
		auto operator=(const FileChainer&) -> FileChainer& = delete;
		
		FileChainer(FileChainer&& rv) noexcept
		: file{rv.file}
		, err{std::move(rv.err)} {
			rv.file = nullptr;
			rv.err = std::string("moved value.");
		}

		auto operator=(FileChainer&& rv) noexcept -> FileChainer& {
			if(this == &rv) return *this;
			if(this->file) std::fclose(this->file);
			this->file = rv.file;
			this->err = std::move(rv.err);
			rv.file = nullptr;
			rv.err = std::string("moved value.");
			return *this;
		}

		~FileChainer() noexcept {
			if(this->file) std::fclose(this->file);
		}

		private:
		FileChainer(std::FILE *const file):
			file{file}
		{}

		public:
		static auto make(const char *const path, const FopenMode mode) noexcept -> std::expected<FileChainer, std::string> {
			constexpr auto conv = [](const FopenMode mode) -> const char * {
				switch(mode) {
					case FopenMode::r: return "r";
					case FopenMode::w: return "w";
					case FopenMode::a: return "a";
					case FopenMode::rp: return "r+";
					case FopenMode::wp: return "w+";
					case FopenMode::ap: return "a+";
					case FopenMode::rb: return "rb";
					case FopenMode::wb: return "wb";
					case FopenMode::ab: return "ab";
					case FopenMode::rbp: return "rb+";
					case FopenMode::wbp: return "wb+";
					case FopenMode::abp: return "ab+";
				}
				std::unreachable();
			};
			const auto mode_str = conv(mode);
			if(const auto file = std::fopen(path, mode_str); file != nullptr) {
				return std::expected<FileChainer, std::string> {
					std::in_place
					, FileChainer{file}
				};
			}
			else {
				return std::expected<FileChainer, std::string> {
					std::unexpect
					, "failed to std::fopen."
				};
			}
		}

		explicit operator bool() const noexcept {
			return !static_cast<bool>(this->err);
		}

		auto is_fail() const noexcept -> bool {
			return static_cast<bool>(this->err);
		}

		auto what() const noexcept -> std::string {
			return this->err ? *this->err : std::string{};
		}

		template<class T_>
		auto read(T_& x) noexcept -> FileChainer& {
			if(this->err) return *this;

			if(std::fread(&x, sizeof(T_), 1, file) != 1) {
				this->err = std::make_optional<std::string>("error: FileChainer::read: std::fread fail or reach eof before enough reading.");
			}

			return *this;
		}

		// こういうときってrange使うべきなんだろうか？
		template<class T_>
		auto read_n(T_ *const dest, const usize n) noexcept -> FileChainer& {
			if(this->err || !dest) return *this;

			if(const auto m = std::fread(dest, sizeof(T_), n, file); m != n) {
				this->err = std::make_optional<std::string>("error: FileChainer::read_n: std::fread fail or reach eof before enough reading.");
			}

			return *this;
		}

		template<class T_>
		auto read_until(T_ *const dest, const usize n, const T_ sentry, usize& read_size) -> FileChainer& {
			if(this->err) return *this;

			usize i = 0;
			for(; i < n; ++i) {
				if(this->read(dest[i]).is_fail()) return *this;
				if(dest[i] == sentry) {
					++i;
					break;
				}
			}
			read_size = i;

			return *this;
		}

		auto seek(const u32 offset, const SeekOrigin origin) -> FileChainer& {
			if(this->err) return *this;

			if(std::fseek(this->file, offset, std::to_underlying(origin)) != 0) {
				this->err = std::make_optional<std::string>("error: FileChainer::seek: std::fseek fail.");
			}

			return *this;
		}

		template<class T_>
		auto write_n(const T_ *const src, const usize n) noexcept -> FileChainer& {
			if(this->err) return *this;
			
			if(const auto m = std::fwrite(src, sizeof(T_), n, this->file); m != n) {
				this->err = std::make_optional<std::string>("error: FileChainer::write_n: std::fwrite fail.");
			}

			return *this;
		}
	};
}