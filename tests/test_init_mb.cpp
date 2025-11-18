#include <cassert>
#include <cmath>
#include <vector>

#include "models/vehiclemodels/init_mb.hpp"
#include "vehicle/parameters_vehicle1.hpp"

namespace vm = velox::models;

namespace {
constexpr double kEpsilon = 1e-6;
}

static void expect_close(double a, double b, double eps = kEpsilon)
{
    assert(std::fabs(a - b) <= eps);
}

int main()
{
    auto params = vm::parameters_vehicle1(VELOX_PARAM_ROOT);

    auto default_state = vm::init_mb({}, params);
    assert(default_state.size() == 29);
    for (double v : default_state) {
        assert(std::isfinite(v));
    }
    // Wheel speeds should be zero when no longitudinal velocity is provided.
    expect_close(default_state[23], 0.0);
    expect_close(default_state[24], 0.0);
    expect_close(default_state[25], 0.0);
    expect_close(default_state[26], 0.0);

    // Partial input is preserved while remaining elements are derived/padded.
    std::vector<double> partial{1.0, -2.0, 0.1};
    auto padded = vm::init_mb(partial, params);
    expect_close(padded[0], 1.0);
    expect_close(padded[1], -2.0);
    expect_close(padded[2], 0.1);
    expect_close(padded[3], 0.0);
    expect_close(padded[4], 0.0);

    return 0;
}
