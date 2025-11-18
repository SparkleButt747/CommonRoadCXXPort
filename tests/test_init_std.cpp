#include <cassert>
#include <cmath>
#include <vector>

#include "models/vehiclemodels/init_std.hpp"
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

    // Empty input should produce a fully padded 9-state vector.
    auto default_state = vm::init_std({}, params);
    assert(default_state.size() == 9);
    for (double v : default_state) {
        assert(std::isfinite(v));
    }
    assert(default_state[3] == 0.0); // velocity
    assert(default_state[7] == 0.0); // front wheel speed
    assert(default_state[8] == 0.0); // rear wheel speed

    // Short inputs are padded but preserve provided elements.
    std::vector<double> partial{1.0, 2.0, 0.1};
    auto padded = vm::init_std(partial, params);
    assert(padded.size() == 9);
    expect_close(padded[0], 1.0);
    expect_close(padded[1], 2.0);
    expect_close(padded[2], 0.1);
    expect_close(padded[3], 0.0); // speed not provided -> zero
    expect_close(padded[7], 0.0);
    expect_close(padded[8], 0.0);

    // Provided base state should drive wheel speed derivation.
    const double v     = 5.0;
    const double beta  = 0.02;
    const double delta = 0.05;
    std::vector<double> full{0.0, 0.0, delta, v, 0.0, 0.0, beta};
    auto derived = vm::init_std(full, params);
    assert(derived.size() == 9);
    const double expected_front = v * std::cos(beta) * std::cos(delta) / params.R_w;
    const double expected_rear  = v * std::cos(beta) / params.R_w;
    expect_close(derived[7], expected_front);
    expect_close(derived[8], expected_rear);

    return 0;
}
