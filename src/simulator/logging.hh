#pragma once

#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

namespace logging = spdlog;

inline auto& init(const std::string& name) {
    spdlog::set_default_logger(spdlog::create<spdlog::sinks::ansicolor_stdout_sink_mt>(name));

    auto& logger = *spdlog::get(name);
    logger.set_pattern("[%Y-%m-%d %H:%M:%S.%f %z] [%n @%t] %^[%l]%$ %v");

    spdlog::cfg::load_env_levels();

    return logger;
}
