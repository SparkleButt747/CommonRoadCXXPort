#include <fstream>
#include <sstream>
#include <string>
#include <vector>

int main()
{
#ifndef VELOX_SOURCE_DIR
#    define VELOX_SOURCE_DIR "."
#endif

    const std::string base = VELOX_SOURCE_DIR;
    const std::vector<std::string> headers = {
        base + "/lib/simulation/simulation_daemon.hpp",
        base + "/lib/telemetry/telemetry.hpp",
    };

    for (const auto& path : headers) {
        std::ifstream file(path);
        if (!file.is_open()) {
            return 1;
        }

        std::ostringstream oss;
        oss << file.rdbuf();
        const std::string contents = oss.str();

        if (contents.find("GearSelection") != std::string::npos) {
            return 2;
        }
    }

    return 0;
}
