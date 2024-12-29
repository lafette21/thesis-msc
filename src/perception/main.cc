#include "logging.hh"
#include "perception.hh"

#include <boost/program_options.hpp>
#include <nova/yaml.hh>

#include <cstdlib>
#include <filesystem>

namespace po = boost::program_options;

using yaml = nova::yaml;


auto parse_args(int argc, char* argv[])
        -> std::optional<po::variables_map>
{
    auto arg_parser = po::options_description("Perception");
    arg_parser.add_options()
        ("config,c", po::value<std::string>()->required()->value_name("FILE"), "Config YAML file")
        ("indir,i", po::value<std::string>()->required()->value_name("DIR"), "Input directory (file name format: `test_fn{num}.xyz`)")
        ("start,s", po::value<std::size_t>()->required()->value_name("START"), "Start of the range")
        ("end,e", po::value<std::size_t>()->required()->value_name("END"), "End of the range")
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
        const auto from = (*args)["start"].as<std::size_t>();
        const auto to = (*args)["end"].as<std::size_t>();
        const auto out_dir = (*args)["outdir"].as<std::string>();

        if (not std::filesystem::exists(out_dir)) {
            if (std::filesystem::create_directory(out_dir)) {
                logging::info("Directory created: {}", out_dir);
            } else {
                logging::error("Failed to create directory: {}", out_dir);
            }
        }

        perception perception(config, from, to, in_dir, out_dir);

        perception.start();

        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        logging::error("Fatal error happened: {}", ex.what());
        return EXIT_FAILURE;
    }
}
