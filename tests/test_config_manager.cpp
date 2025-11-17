#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

#include "io/config_manager.hpp"
#include "simulation/model_timing.hpp"

namespace vio  = velox::io;
namespace vsim = velox::simulation;

namespace {

std::filesystem::path make_temp_dir(const std::string& name)
{
    const auto root = std::filesystem::temp_directory_path() / name;
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    return root;
}

void write_file(const std::filesystem::path& path, const std::string& contents)
{
    std::ofstream out(path);
    out << contents;
}

} // namespace

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

    const auto temp_root = make_temp_dir("velox_config_missing");

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
        write_file(invalid_root / "powertrain.yaml",
                   "max_drive_torque: 1\n"
                   "max_regen_torque: 1\n"
                   "max_power: 1\n"
                   "drive_efficiency: 0.5\n"
                   "regen_efficiency: 0.5\n"
                   "min_soc: 0.8\n"
                   "max_soc: 0.2\n"
                   "initial_soc: 0.5\n"
                   "battery_capacity_kwh: 10\n");
    }

    bool invalid_range_threw = false;
    try {
        vio::ConfigManager invalid(invalid_root);
        invalid.load_powertrain_config();
    } catch (const std::exception&) {
        invalid_range_threw = true;
    }
    assert(invalid_range_threw && "Invalid SOC bounds should throw");

    std::mt19937                          rng{1234};
    std::uniform_real_distribution<double> invalid_dt(-1.0, 0.09);
    for (int i = 0; i < 25; ++i) {
        const auto timing_root = temp_root / ("timing_" + std::to_string(i));
        std::filesystem::create_directories(timing_root);
        const double bad_nominal = invalid_dt(rng);
        const double bad_max     = bad_nominal - 0.5;
        write_file(timing_root / "model_timing.yaml",
                   "st:\n"
                   "  nominal_dt: "
                       + std::to_string(bad_nominal)
                       + "\n"
                         "  max_dt: "
                       + std::to_string(bad_max)
                       + "\n");

        vio::ConfigManager timing_mgr{timing_root};
        bool               bad_dt_threw = false;
        try {
            timing_mgr.load_model_timing(vsim::ModelType::ST);
        } catch (const std::exception& ex) {
            bad_dt_threw = (std::string{ex.what()}.find("nominal_dt") != std::string::npos)
                || (std::string{ex.what()}.find("max_dt") != std::string::npos);
        }
        assert(bad_dt_threw && "Timing validation should reject invalid dt ranges");
    }

    const auto override_root = make_temp_dir("velox_safety_override");
    write_file(override_root / "low_speed_safety.yaml",
               "engage_speed: 0.1\nrelease_speed: 0.2\nyaw_rate_limit: 0.5\nslip_angle_limit: 0.3\nstop_speed_epsilon: 0.05\n");
    write_file(override_root / "low_speed_safety_st.yaml",
               "engage_speed: 1.1\nrelease_speed: 1.2\nyaw_rate_limit: 0.25\nslip_angle_limit: 0.15\nstop_speed_epsilon: 0.01\n");

    vio::ConfigManager override_mgr{override_root};
    const auto         default_cfg  = override_mgr.load_low_speed_safety_config(vsim::ModelType::KS_REAR);
    const auto         override_cfg = override_mgr.load_low_speed_safety_config(vsim::ModelType::ST);
    assert(default_cfg.engage_speed == 0.1 && override_cfg.engage_speed == 1.1);
    assert(default_cfg.release_speed == 0.2 && override_cfg.release_speed == 1.2);

    const auto steering_root = make_temp_dir("velox_steering_missing");
    write_file(steering_root / "steering.yaml",
               "wheel:\n  rate_limit: 1.0\n  damping: 0.1\n  stiffness: 0.2\nfinal:\n  kp: 1.0\n  ki: 0.0\n  kd: 0.0\n");

    bool missing_field_threw = false;
    try {
        vio::ConfigManager missing_field_mgr{steering_root};
        missing_field_mgr.load_steering_config();
    } catch (const std::exception&) {
        missing_field_threw = true;
    }
    assert(missing_field_threw && "Missing steering fields should be reported");

    std::cout << "Config manager tests passed\n";
    return 0;
}

