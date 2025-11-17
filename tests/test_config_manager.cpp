#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "io/config_manager.hpp"

namespace vio  = velox::io;
namespace vsim = velox::simulation;

int main()
{
    try {
        vio::ConfigManager configs{};
        const auto brake_cfg = configs.load_brake_config();
        assert(brake_cfg.max_force > 0.0);
        const auto timing = configs.load_model_timing(vsim::ModelType::ST);
        assert(timing.max_dt >= timing.nominal_dt);
    } catch (const std::exception& e) {
        std::cerr << "Default load failed: " << e.what() << '\n';
        return 1;
    }

    const auto temp_root = std::filesystem::temp_directory_path() / "velox_config_missing";
    std::filesystem::remove_all(temp_root);
    std::filesystem::create_directories(temp_root);

    bool missing_threw = false;
    try {
        vio::ConfigManager missing(temp_root);
        std::cout << "Attempting to load from: " << temp_root << '\n';
        missing.load_brake_config();
    } catch (const std::exception&) {
        missing_threw = true;
    }
    assert(missing_threw && "Missing brake config should throw");

    const auto invalid_root = temp_root / "invalid";
    std::filesystem::create_directories(invalid_root);
    {
        std::ofstream out(invalid_root / "powertrain.yaml");
        out << "max_drive_torque: 1\n"
               "max_regen_torque: 1\n"
               "max_power: 1\n"
               "drive_efficiency: 0.5\n"
               "regen_efficiency: 0.5\n"
               "min_soc: 0.8\n"
               "max_soc: 0.2\n"
               "initial_soc: 0.5\n"
               "battery_capacity_kwh: 10\n";
    }

    bool invalid_range_threw = false;
    try {
        vio::ConfigManager invalid(invalid_root);
        invalid.load_powertrain_config();
    } catch (const std::invalid_argument&) {
        invalid_range_threw = true;
    }
    assert(invalid_range_threw && "Invalid SOC bounds should throw");

    std::cout << "Config manager tests passed\n";
    return 0;
}

