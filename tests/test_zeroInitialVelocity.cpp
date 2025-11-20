// tests/test_zeroInitialVelocity.cpp
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <vector>

#include "vehicle/parameters_vehicle2.hpp"

#include "models/vehiclemodels/vehicle_dynamics_st.hpp"
#include "models/vehiclemodels/vehicle_dynamics_mb.hpp"

#include "models/vehiclemodels/init_st.hpp"
#include "models/vehiclemodels/init_mb.hpp"

namespace vm = velox::models;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static void fail(const char* test_name,
                 const char* case_name,
                 std::size_t index,
                 double got,
                 double expected,
                 double diff,
                 double tol)
{
    std::cerr << "TEST FAILED: " << test_name
              << " (" << case_name << ") at index " << index
              << " expected = " << std::setprecision(17) << expected
              << " got = " << got
              << " |diff| = " << diff
              << " > tol = " << tol
              << std::endl;
    std::exit(1);
}

static void check_vec(const std::vector<double>& got,
                      const std::vector<double>& expected,
                      double tol,
                      const char* test_name,
                      const char* case_name)
{
    if (got.size() != expected.size()) {
        std::cerr << "TEST FAILED: " << test_name << " (" << case_name << ") "
                  << "size mismatch: got " << got.size()
                  << ", expected " << expected.size() << std::endl;
        std::exit(1);
    }

    for (std::size_t i = 0; i < got.size(); ++i) {
        double diff = got[i] - expected[i];
        if (std::fabs(diff) > tol) {
            fail(test_name, case_name, i, got[i], expected[i], std::fabs(diff), tol);
        }
    }
}

// Fixed-step Runge-Kutta 4 integrator (matches Python reference behaviour better
// than explicit Euler for stiff portions of the MB dynamics).
template <typename DynamicsFunc>
std::vector<double> integrate_rk4(DynamicsFunc f,
                                  const std::vector<double>& x0,
                                  const std::vector<double>& u,
                                  const vm::VehicleParameters& p,
                                  double t_start,
                                  double t_final,
                                  double dt)
{
    std::vector<double> x = x0;
    const int steps = static_cast<int>((t_final - t_start) / dt);

    for (int i = 0; i < steps; ++i) {
        auto k1 = f(x, u, p);
        if (k1.size() != x.size()) {
            std::cerr << "Dimension mismatch in dynamics: k1.size() = "
                      << k1.size() << ", x.size() = " << x.size() << std::endl;
            std::exit(1);
        }

        std::vector<double> x_tmp(x.size());

        for (std::size_t j = 0; j < x.size(); ++j) {
            x_tmp[j] = x[j] + 0.5 * dt * k1[j];
        }
        auto k2 = f(x_tmp, u, p);
        if (k2.size() != x.size()) {
            std::cerr << "Dimension mismatch in dynamics: k2.size() = "
                      << k2.size() << ", x.size() = " << x.size() << std::endl;
            std::exit(1);
        }

        for (std::size_t j = 0; j < x.size(); ++j) {
            x_tmp[j] = x[j] + 0.5 * dt * k2[j];
        }
        auto k3 = f(x_tmp, u, p);
        if (k3.size() != x.size()) {
            std::cerr << "Dimension mismatch in dynamics: k3.size() = "
                      << k3.size() << ", x.size() = " << x.size() << std::endl;
            std::exit(1);
        }

        for (std::size_t j = 0; j < x.size(); ++j) {
            x_tmp[j] = x[j] + dt * k3[j];
        }
        auto k4 = f(x_tmp, u, p);
        if (k4.size() != x.size()) {
            std::cerr << "Dimension mismatch in dynamics: k4.size() = "
                      << k4.size() << ", x.size() = " << x.size() << std::endl;
            std::exit(1);
        }

        for (std::size_t j = 0; j < x.size(); ++j) {
            x[j] += (dt / 6.0) * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]);
            if (x.size() >= 27 && j >= 23 && j <= 26 && x[j] < 0.0) {
                x[j] = 0.0;
            }
        }
    }

    return x;
}

static std::vector<double> dyn_ST(const std::vector<double>& x,
                                  const std::vector<double>& u,
                                  const vm::VehicleParameters& p)
{
    return vm::vehicle_dynamics_st(x, u, p);
}

static std::vector<double> dyn_MB(const std::vector<double>& x,
                                  const std::vector<double>& u,
                                  const vm::VehicleParameters& p)
{
    return vm::vehicle_dynamics_mb(x, u, p);
}

// -----------------------------------------------------------------------------
// Main: port of Python test_zeroInitialVelocity()
// -----------------------------------------------------------------------------

int main()
{
    const char* TEST_NAME = "test_zeroInitialVelocity";

    // parameters
    vm::VehicleParameters p = vm::parameters_vehicle2();
    const double g = 9.81;

    // time vector: t = arange(0, 1, 1e-4)
    const double t_start = 0.0;
    const double t_final = 1.0;
    const double dt      = 1e-4;

    // core initial state [sx0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]
    const double sx0     = 0.0;
    const double sy0     = 0.0;
    const double delta0  = 0.0;
    const double vel0    = 0.0;
    const double Psi0    = 0.0;
    const double dotPsi0 = 0.0;
    const double beta0   = 0.0;

    std::vector<double> initialState{
        sx0, sy0, delta0, vel0, Psi0, dotPsi0, beta0
    };

    // initial states via init_* (matching Python)
    std::vector<double> x0_ST = vm::init_st(initialState);
    std::vector<double> x0_MB = vm::init_mb(initialState, p);

    // -------------------------------------------------------------------------
    // 1) Rolling car, u = [0, 0]
    // -------------------------------------------------------------------------
    std::vector<double> u_roll{0.0, 0.0};

    auto x_roll    = integrate_rk4(dyn_MB, x0_MB, u_roll, p, t_start, t_final, dt);
    auto x_roll_st = integrate_rk4(dyn_ST, x0_ST, u_roll, p, t_start, t_final, dt);

    std::vector<double> x_roll_gt{
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        2.9396925632701572e-18, 1.0382619602630607e-18, -0.001383413646355998,
        -0.0020336342166179544, -6.8100035962383285e-18, 0.017624852018447854,
        0.007165545259572071, 2.499120748142407e-18, -1.4334324855610345e-18,
        -7.3404080848722666e-18, 0.018676373794485092, 0.00037525175879187217,
        1.2669696045979021e-18, -2.8465346036555629e-18, -7.3631025851755168e-18,
        0.015462186524627917, 0.00002516250917098079, 1.7110213358664119,
        1.7110213358664119, 0.0, 0.0,
        1.1218513664139043e-18, 1.4834796531562799e-18
    };

    check_vec(x_roll,    x_roll_gt, 1e-2, TEST_NAME, "roll_MB");
    check_vec(x_roll_st, x0_ST,     0.0,  TEST_NAME, "roll_ST_constant");

    // -------------------------------------------------------------------------
    // 2) Decelerating car, v_delta = 0, acc = -0.7*g
    // -------------------------------------------------------------------------
    {
        const double v_delta = 0.0;
        const double acc     = -0.7 * g;
        std::vector<double> u_dec{v_delta, acc};

        auto x_dec    = integrate_rk4(dyn_MB, x0_MB, u_dec, p, t_start, t_final, dt);
        auto x_dec_st = integrate_rk4(dyn_ST, x0_ST, u_dec, p, t_start, t_final, dt);

        std::vector<double> x_dec_gt{
            1.8225618646966624, -0.0086273097089525977, 0.0,
            -3.4554290247963668, 0.00047929531592577661, -1.5920968922503272e-05,
            -0.00025302742748178734, -0.00011973072674828772, -0.018779685455296551,
            -0.0090084648185682851, 0.017283043089364594, 0.018135231336119616,
            0.0086135837859677312, -4.481562597218733e-05, -1.4272890015767374e-05,
            0.017349174844723529, 0.021532679755773425, 0.00038187032873207187,
            -3.3920811271631363e-05, -9.4784375662071155e-06, 0.017377021416567273,
            0.012706419736429759, 9.4015391546569673e-06, 0.0, 0.0, 0.0, 0.0,
            -0.00020661575881509941, -0.00017869393157893184
        };

        std::vector<double> x_dec_st_gt{
            -3.4335000000000013, 0.0000000000000000, 0.0000000000000000,
            -6.8670000000000018, 0.0000000000000000, 0.0000000000000000,
            0.0000000000000000
        };

        check_vec(x_dec,    x_dec_gt,    1e-2, TEST_NAME, "dec_MB");
        check_vec(x_dec_st, x_dec_st_gt, 1e-2, TEST_NAME, "dec_ST");
    }

    // -------------------------------------------------------------------------
    // 3) Accelerating car, v_delta = 0.15, acc = 0.63*g
    // -------------------------------------------------------------------------
    {
        const double v_delta = 0.15;
        const double acc     = 0.63 * g;
        std::vector<double> u_acc{v_delta, acc};

        auto x_acc    = integrate_rk4(dyn_MB, x0_MB, u_acc, p, t_start, t_final, dt);
        auto x_acc_st = integrate_rk4(dyn_ST, x0_ST, u_acc, p, t_start, t_final, dt);

        std::vector<double> x_acc_gt{
            1.6871562929572532, 0.0042854001980273818, 0.14999999999998667,
            3.1974132719678994, 0.33806720788220607, 0.89130846925333573,
            -0.018585793734759059, -0.05569523922934138, 0.014166803684133662,
            0.010814605916134869, -0.62911782106847891, 0.017269790314292637,
            0.0025945395929668643, -0.0042181329490571275, -0.011573377836930512,
            0.45274405656671479, 0.016136574608443133, -0.0012368703361228014,
            -0.002363031148676171, -0.0072193161379929842, -1.8638044064961889,
            0.017959249779534586, 0.0010263382407978082, 11.132619776576281,
            7.5830653080368968, 308.32216462068976, 310.90017338219224,
            -0.019679020182265954, -0.0083587135435600583
        };

        std::vector<double> x_acc_st_gt{
            3.0731976046859715, 0.2869835398304389, 0.1500000000000000,
            6.1802999999999999, 0.1097747074946325, 0.3248268063223301,
            0.0697547542798040
        };

        check_vec(x_acc,    x_acc_gt,    1e-2, TEST_NAME, "acc_MB");
        check_vec(x_acc_st, x_acc_st_gt, 1e-2, TEST_NAME, "acc_ST");
    }

    // -------------------------------------------------------------------------
    // 4) Steering to the left, v_delta = 0.15, acc = 0
    // -------------------------------------------------------------------------
    {
        const double v_delta = 0.15;
        const double acc     = 0.0;
        std::vector<double> u_left{v_delta, acc};

        auto x_left    = integrate_rk4(dyn_MB, x0_MB, u_left, p, t_start, t_final, dt);
        auto x_left_st = integrate_rk4(dyn_ST, x0_ST, u_left, p, t_start, t_final, dt);

        std::vector<double> x_left_gt{
            0.0, 0.0, 0.14999999999998667,
            0.0, 0.0, 0.0,
            0.00030874729123515774, 0.011699177588952989, -0.0013796623269185146,
            -0.0019195672722097301, -0.0066065475420350496, 0.017624825341632389,
            0.007164007130114107, 0.00015051447265641363, 0.0091936447322982333,
            -0.017987682527725091, 0.018675048631672228, 0.00035569258254374372,
            0.00016978794795261313, 0.0016007735062562938, -0.01772584741880711,
            0.015463728064918913, 4.9390231495498064e-05, 1.7055154881400041,
            1.7163126649152141, 0.0, 0.0,
            0.00023660612929119265, 0.00037315260622417255
        };

        std::vector<double> x_left_st_gt{
            0.0, 0.0, 0.14999999999998667,
            0.0, 0.0, 0.0,
            0.083374602676255474
        };

        check_vec(x_left,    x_left_gt,    1e-2, TEST_NAME, "left_MB");
        check_vec(x_left_st, x_left_st_gt, 1e-2, TEST_NAME, "left_ST");
    }

    std::cout << "All zero-initial-velocity tests PASSED." << std::endl;
    return 0;
}
