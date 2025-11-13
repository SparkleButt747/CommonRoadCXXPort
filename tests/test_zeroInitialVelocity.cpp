// tests/test_zeroInitialVelocity.cpp
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <vector>

#include "vehicle/parameters_vehicle2.hpp"

#include "vehiclemodels/vehicle_dynamics_ks.hpp"
#include "vehiclemodels/vehicle_dynamics_st.hpp"
#include "vehiclemodels/vehicle_dynamics_mb.hpp"

#include "vehiclemodels/init_ks.hpp"
#include "vehiclemodels/init_st.hpp"
#include "vehiclemodels/init_mb.hpp"

namespace vm = vehiclemodels;

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

// Simple fixed-step explicit Euler integrator
template <typename DynamicsFunc>
std::vector<double> integrate_euler(DynamicsFunc f,
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
        auto dx = f(x, u, p);
        if (dx.size() != x.size()) {
            std::cerr << "Dimension mismatch in dynamics: dx.size() = "
                      << dx.size() << ", x.size() = " << x.size() << std::endl;
            std::exit(1);
        }
        for (std::size_t j = 0; j < x.size(); ++j) {
            x[j] += dt * dx[j];
        }
    }

    return x;
}

// Convenience wrappers
static std::vector<double> dyn_KS(const std::vector<double>& x,
                                  const std::vector<double>& u,
                                  const vm::VehicleParameters& p)
{
    return vm::vehicle_dynamics_ks(x, u, p);
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
    std::vector<double> x0_KS = vm::init_ks(initialState);
    std::vector<double> x0_ST = vm::init_st(initialState);
    std::vector<double> x0_MB = vm::init_mb(initialState, p);

    // -------------------------------------------------------------------------
    // 1) Rolling car, u = [0, 0]
    // -------------------------------------------------------------------------
    std::vector<double> u_roll{0.0, 0.0};

    auto x_roll    = integrate_euler(dyn_MB, x0_MB, u_roll, p, t_start, t_final, dt);
    auto x_roll_st = integrate_euler(dyn_ST, x0_ST, u_roll, p, t_start, t_final, dt);
    auto x_roll_ks = integrate_euler(dyn_KS, x0_KS, u_roll, p, t_start, t_final, dt);

    std::vector<double> x_roll_gt{
        0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
        0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
        -0.0000000003174207, 0.0000000848065981, -0.0013834133396573,
        -0.0020336367252011, -0.0000000247286655, 0.0176248518072475,
        0.0071655470428753, 0.0000000006677358, -0.0000001709775865,
        0.0000001839820148, 0.0186763737562366, 0.0003752526345970,
        0.0000000006728055, -0.0000001734436431, 0.0000001850020879,
        0.0154621865353889, 0.0000251622262094, -0.0000174466440656,
        -0.0000174466440656, -0.0000014178345014, -0.0000014178345014,
        0.0000000008088692, 0.0000000008250785
    };

    check_vec(x_roll,    x_roll_gt, 1e-2, TEST_NAME, "roll_MB");
    check_vec(x_roll_st, x0_ST,     0.0,  TEST_NAME, "roll_ST_constant");
    check_vec(x_roll_ks, x0_KS,     0.0,  TEST_NAME, "roll_KS_constant");

    // -------------------------------------------------------------------------
    // 2) Decelerating car, v_delta = 0, acc = -0.7*g
    // -------------------------------------------------------------------------
    {
        const double v_delta = 0.0;
        const double acc     = -0.7 * g;
        std::vector<double> u_dec{v_delta, acc};

        auto x_dec    = integrate_euler(dyn_MB, x0_MB, u_dec, p, t_start, t_final, dt);
        auto x_dec_st = integrate_euler(dyn_ST, x0_ST, u_dec, p, t_start, t_final, dt);
        auto x_dec_ks = integrate_euler(dyn_KS, x0_KS, u_dec, p, t_start, t_final, dt);

        std::vector<double> x_dec_gt{
            3.9830932439714273, -0.0601543816394762, 0.0000000000000000,
            -8.0013986587693893, -0.0026467910011602, -0.0053025639381130,
            -0.0019453336082831, -0.0002270008481486, -0.0431740570135473,
            -0.0305313864800163, 0.1053033709671264, 0.0185102262795369,
            0.0137681838589757, -0.0003400843778018, -0.0000161129034355,
            0.0994502177784092, 0.0256268504637763, 0.0034700280714177,
            -0.0002562443897593, -0.0000034699487925, 0.1128675292571426,
            0.0086968977905411, -0.0020987862166353, -0.0000183158385631,
            -0.0000183158385631, -0.0000095073736467, -0.0000095073736467,
            -0.0016872664171374, -0.0012652511246015
        };

        std::vector<double> x_dec_st_gt{
            -3.4335000000000013, 0.0000000000000000, 0.0000000000000000,
            -6.8670000000000018, 0.0000000000000000, 0.0000000000000000,
            0.0000000000000000
        };

        std::vector<double> x_dec_ks_gt{
            -3.4335000000000013, 0.0000000000000000, 0.0000000000000000,
            -6.8670000000000018, 0.0000000000000000
        };

        check_vec(x_dec,    x_dec_gt,    1e-2, TEST_NAME, "dec_MB");
        check_vec(x_dec_st, x_dec_st_gt, 1e-2, TEST_NAME, "dec_ST");
        check_vec(x_dec_ks, x_dec_ks_gt, 1e-2, TEST_NAME, "dec_KS");
    }

    // -------------------------------------------------------------------------
    // 3) Accelerating car, v_delta = 0.15, acc = 0.63*g
    // -------------------------------------------------------------------------
    {
        const double v_delta = 0.15;
        const double acc     = 0.63 * g;
        std::vector<double> u_acc{v_delta, acc};

        auto x_acc    = integrate_euler(dyn_MB, x0_MB, u_acc, p, t_start, t_final, dt);
        auto x_acc_st = integrate_euler(dyn_ST, x0_ST, u_acc, p, t_start, t_final, dt);
        auto x_acc_ks = integrate_euler(dyn_KS, x0_KS, u_acc, p, t_start, t_final, dt);

        std::vector<double> x_acc_gt{
            1.6869441956852231, 0.0041579276718349, 0.1500000000000001,
            3.1967387404602654, 0.3387575860582390, 0.8921302762726965,
            -0.0186007698209413, -0.0556855538608812, 0.0141668816602887,
            0.0108112584162600, -0.6302339461329982, 0.0172692751292486,
            0.0025948291288222, -0.0042209020256358, -0.0115749221900647,
            0.4525764527765288, 0.0161366380049974, -0.0012354790918115,
            -0.0023647389844973, -0.0072210348979615, -1.8660984955372673,
            0.0179591511062951, 0.0010254111038481, 11.1322413877606117,
            7.5792605585643713, 308.3079237740076906, 310.8801727728298374,
            -0.0196922024889714, -0.0083685253175425
        };

        std::vector<double> x_acc_st_gt{
            3.0731976046859715, 0.2869835398304389, 0.1500000000000000,
            6.1802999999999999, 0.1097747074946325, 0.3248268063223301,
            0.0697547542798040
        };

        std::vector<double> x_acc_ks_gt{
            3.0845676868494927, 0.1484249221523042, 0.1500000000000000,
            6.1803000000000017, 0.1203664469224163
        };

        check_vec(x_acc,    x_acc_gt,    1e-2, TEST_NAME, "acc_MB");
        check_vec(x_acc_st, x_acc_st_gt, 1e-2, TEST_NAME, "acc_ST");
        check_vec(x_acc_ks, x_acc_ks_gt, 1e-2, TEST_NAME, "acc_KS");
    }

    // -------------------------------------------------------------------------
    // 4) Steering to the left, v_delta = 0.15, acc = 0
    // -------------------------------------------------------------------------
    {
        const double v_delta = 0.15;
        const double acc     = 0.0;
        std::vector<double> u_left{v_delta, acc};

        auto x_left    = integrate_euler(dyn_MB, x0_MB, u_left, p, t_start, t_final, dt);
        auto x_left_st = integrate_euler(dyn_ST, x0_ST, u_left, p, t_start, t_final, dt);
        auto x_left_ks = integrate_euler(dyn_KS, x0_KS, u_left, p, t_start, t_final, dt);

        std::vector<double> x_left_gt{
            0.0000000000000000, 0.0000000000000000, 0.1500000000000000,
            0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
            0.0003021160057306, 0.0115474648881108, -0.0013797955031689,
            -0.0019233204598741, -0.0065044050021887, 0.0176248291065725,
            0.0071641239008779, 0.0001478513434683, 0.0092020911982902,
            -0.0178028732533553, 0.0186751057310096, 0.0003566948613572,
            0.0001674970785214, 0.0015871955172538, -0.0175512251679294,
            0.0154636630992985, 0.0000482191918813, -0.0000173442953338,
            -0.0000174708138706, -0.0000014178345014, -0.0000014178345014,
            0.0002293337149155, 0.0003694012334077
        };

        std::vector<double> x_left_st_gt{
            0.0000000000000000, 0.0000000000000000, 0.1500000000000000,
            0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
            0.0000000000000000
        };

        std::vector<double> x_left_ks_gt{
            0.0000000000000000, 0.0000000000000000, 0.1500000000000000,
            0.0000000000000000, 0.0000000000000000
        };

        check_vec(x_left,    x_left_gt,    1e-2, TEST_NAME, "left_MB");
        check_vec(x_left_st, x_left_st_gt, 1e-2, TEST_NAME, "left_ST");
        check_vec(x_left_ks, x_left_ks_gt, 1e-2, TEST_NAME, "left_KS");
    }

    std::cout << "All zero-initial-velocity tests PASSED." << std::endl;
    return 0;
}
