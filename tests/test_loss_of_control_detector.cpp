#include <cassert>
#include <iostream>
#include <vector>

#include "simulation/loss_of_control_detector.hpp"

namespace vsim = velox::simulation;

vsim::LossOfControlDetectorConfig make_config()
{
    vsim::LossOfControlDetectorConfig cfg{};
    cfg.yaw_rate.magnitude_threshold   = 0.4;
    cfg.yaw_rate.rate_threshold        = 6.0;
    cfg.slip_angle.magnitude_threshold = 0.3;
    cfg.slip_angle.rate_threshold      = 4.0;
    cfg.lateral_accel.magnitude_threshold = 3.0;
    cfg.lateral_accel.rate_threshold      = 5.0;
    cfg.slip_ratio.magnitude_threshold    = 0.15;
    cfg.slip_ratio.rate_threshold         = 5.0;
    cfg.validate();
    return cfg;
}

void test_slip_spike()
{
    auto cfg = make_config();
    vsim::LossOfControlDetector detector(cfg);

    std::vector<double> wheel_slip(4, 0.02);
    detector.update(0.01, 0.05, 0.1, 0.5, wheel_slip);

    wheel_slip[0] = 0.6;
    const double severity = detector.update(0.01, 0.05, 0.1, 0.5, wheel_slip);
    assert(severity > 0.1);
}

void test_yaw_spike()
{
    auto cfg = make_config();
    vsim::LossOfControlDetector detector(cfg);

    std::vector<double> wheel_slip(4, 0.0);
    detector.update(0.02, 0.1, 0.05, 0.2, wheel_slip);

    const double severity = detector.update(0.02, 1.0, 0.3, 0.2, wheel_slip);
    assert(severity > 0.0);
    assert(detector.severity() == severity);
}

void test_recovery_resets_severity()
{
    auto cfg = make_config();
    vsim::LossOfControlDetector detector(cfg);

    std::vector<double> wheel_slip(4, 0.01);
    detector.update(0.01, 0.05, 0.05, 0.1, wheel_slip);

    wheel_slip[1] = 0.5;
    const double spike = detector.update(0.01, 0.05, 0.05, 0.1, wheel_slip);
    assert(spike > 0.0);

    wheel_slip.assign(4, 0.01);
    const double recovered = detector.update(0.01, 0.05, 0.05, 0.1, wheel_slip);
    assert(recovered == 0.0);
    assert(detector.severity() == 0.0);
}

int main()
{
    test_slip_spike();
    test_yaw_spike();
    test_recovery_resets_severity();
    std::cout << "Loss of control detector tests passed\n";
    return 0;
}
