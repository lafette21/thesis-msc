#include "logging.hh"
#include "slam.hh"

#include <boost/program_options.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <nova/io.hh>
#include <nova/yaml.hh>
#include <range/v3/algorithm.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>

namespace po = boost::program_options;

using yaml = nova::yaml;


void validate_format(const std::string& format) {
    const auto valid_formats = std::vector<std::string>{ "ply", "xyz" };

    if (not ranges::contains(valid_formats, format)) {
        throw po::validation_error(po::validation_error::invalid_option_value, "format", format);
    }
}


auto parse_args(int argc, char* argv[])
        -> std::optional<po::variables_map>
{
    auto arg_parser = po::options_description("SLAM");
    arg_parser.add_options()
        ("config,c", po::value<std::string>()->required()->value_name("FILE"), "Config YAML file")
        ("indir,i", po::value<std::string>()->required()->value_name("DIR"), "Input directory (file name format: `test_fn{num}.xyz`)")
        ("odometry", po::value<std::string>()->required()->value_name("ODOMETRY"), "File containing the odometry measurements")
        ("format,f", po::value<std::string>()->required()->value_name("ply|xyz")->notifier(validate_format)->default_value("ply"), "Output format")
        ("outdir,o", po::value<std::string>()->required()->value_name("DIR")->default_value("out"), "Output directory")
        ("help,h", "Show this help");

    po::variables_map args;
    po::store(po::parse_command_line(argc, argv, arg_parser), args);

    if (args.contains("help")) {
        std::cerr << arg_parser << std::endl;
        return std::nullopt;
    }

    args.notify();

    return args;
}


int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv[]) {
    [[maybe_unused]] auto& logger = init("logger");

    try {
        const auto args = parse_args(argc, argv);

        if (not args.has_value()) {
            return EXIT_SUCCESS;
        }

        const yaml config(nova::read_file(std::filesystem::path((*args)["config"].as<std::string>()).string()).value());
        const auto in_dir = (*args)["indir"].as<std::string>();
        const auto odometry = (*args)["odometry"].as<std::string>();
        const auto out_dir = (*args)["outdir"].as<std::string>();
        const auto format = (*args)["format"].as<std::string>();

        if (not std::filesystem::exists(out_dir)) {
            if (std::filesystem::create_directory(out_dir)) {
                logging::info("Directory created: {}", out_dir);
            } else {
                logging::error("Failed to create directory: {}", out_dir);
            }
        }

        slam slam(config, in_dir, odometry, out_dir, format);

        slam.do_();

        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        logging::error("Fatal error happened: {}", ex.what());
        return EXIT_FAILURE;
    }
}
