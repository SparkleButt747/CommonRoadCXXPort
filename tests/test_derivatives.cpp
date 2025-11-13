#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <vector>

#include "vehiclemodels/vehicle_dynamics_ks.hpp"
#include "vehiclemodels/vehicle_dynamics_kst.hpp"
#include "vehiclemodels/vehicle_dynamics_st.hpp"
#include "vehiclemodels/vehicle_dynamics_std.hpp"
#include "vehiclemodels/vehicle_dynamics_mb.hpp"

#include "vehicle/parameters_vehicle2.hpp"
#include "vehicle/parameters_vehicle4.hpp"

namespace vm = vehiclemodels;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static void fail(const char* test_name,
                 std::size_t index,
                 double got,
                 double expected,
                 double diff,
                 double tol)
{
    std::cerr << "TEST FAILED: " << test_name
              << " at index " << index
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
                      const char* test_name)
{
    if (got.size() != expected.size()) {
        std::cerr << "TEST FAILED: " << test_name
                  << " size mismatch: got " << got.size()
                  << ", expected " << expected.size() << std::endl;
        std::exit(1);
    }

    for (std::size_t i = 0; i < got.size(); ++i) {
        double diff = got[i] - expected[i];
        if (std::fabs(diff) > tol) {
            fail(test_name, i, got[i], expected[i], std::fabs(diff), tol);
        }
    }
}

// -----------------------------------------------------------------------------
// Individual tests (direct port of Python unit tests)
// -----------------------------------------------------------------------------

static void test_ks(const vm::VehicleParameters& p, const std::vector<double>& u)
{
    // state values
    std::vector<double> x_ks{
        3.9579422297936526,
        0.0391650102771405,
        0.0378491427211811,
        16.3546957860883566,
        0.0294717351052816
    };

    // ground truth
    std::vector<double> f_ks_gt{
        16.3475935934250209,
        0.4819314886013121,
        0.1500000000000000,
        5.1464424102339752,
        0.2401426578627629
    };

    auto f_ks = vm::vehicle_dynamics_ks(x_ks, u, p);
    // Python used places=14 -> ~1e-14. Be strict but give a hair of slack.
    check_vec(f_ks, f_ks_gt, 1e-13, "vehicle_dynamics_ks");
}

static void test_kst(const std::vector<double>& u)
{
    // state values
    std::vector<double> x_kst{
        3.9579422297936526,
        0.0391650102771405,
        0.0378491427211811,
        16.3546957860883566,
        0.0294717351052816,
        0.0294717351052816
    };

    // ground truth
    std::vector<double> f_kst_gt{
        16.3475935934250209,
        0.4819314886013121,
        0.1500000000000000,
        5.501539201758522,
        0.1720297150523055,
        -0.23152742969444276
    };

    auto p4 = vm::parameters_vehicle4();
    auto f_kst = vm::vehicle_dynamics_kst(x_kst, u, p4);
    check_vec(f_kst, f_kst_gt, 1e-13, "vehicle_dynamics_kst");
}

static void test_st(const vm::VehicleParameters& p, const std::vector<double>& u)
{
    // state values
    std::vector<double> x_st{
        2.0233348142065677,
        0.0041907137716636,
        0.0197545248559617,
        15.7216236334290116,
        0.0025857914776859,
        0.0529001056654038,
        0.0033012170610298
    };

    // ground truth
    std::vector<double> f_st_gt{
        15.7213512030862397,
        0.0925527979719355,
        0.1500000000000000,
        5.3536773276413925,
        0.0529001056654038,
        0.6435589397748606,
        0.0313297971641291
    };

    auto f_st = vm::vehicle_dynamics_st(x_st, u, p);
    check_vec(f_st, f_st_gt, 1e-13, "vehicle_dynamics_st");
}

static void test_std(const vm::VehicleParameters& p, const std::vector<double>& u)
{
    // state values
    std::vector<double> x_std{
        2.0233348142065677,
        0.0041907137716636,
        0.0197545248559617,
        15.7216236334290116,
        0.0025857914776859,
        0.0529001056654038,
        0.0033012170610298,
        53.6551710641082451,
        56.7917911784219598
    };

    // ground truth
    std::vector<double> f_std_gt{
        15.72135120308624,
        0.09255279797193551,
        0.15,
        11.359186518121511,
        0.0529001056654038,
        0.5870894694428362,
        -0.007910389538735838,
        -1403.4833546513996,
        72.25313872307531
    };

    auto f_std = vm::vehicle_dynamics_std(x_std, u, p);
    check_vec(f_std, f_std_gt, 1e-13, "vehicle_dynamics_std");
}

static void test_mb(const vm::VehicleParameters& p, const std::vector<double>& u)
{
    // state values
    std::vector<double> x_mb{
        10.8808433066274794,
        0.5371850187869442,
        0.0980442671005920,
        18.0711398687457745,
        0.1649995631003776,
        0.6158755000936103,
        -0.1198403612262477,
        -0.2762672756169581,
        0.0131909920269115,
        -0.0718483683742141,
        -0.3428324595054725,
        0.0103233373083297,
        -0.0399590564140291,
        -0.0246468320579360,
        -0.0551575051990853,
        0.5798277643297529,
        0.0172059354801703,
        0.0067890113155477,
        -0.0184269459410162,
        -0.0408207136116175,
        -1.0064484829203018,
        0.0166808347900582,
        -0.0049188492004049,
        53.6551710641082451,
        50.7045242506744316,
        56.7917911784219598,
        200.7079633169796296,
        -0.0939969123691911,
        -0.0881514621614376
    };

    // ground truth
    std::vector<double> f_mb_gt{
        17.8820162482414098,
        2.6300428035858809,
        0.1500000000000000,
        2.7009396644636450,
        0.6158755000936103,
        1.3132879472301846,
        -0.2762672756169581,
        -0.1360581472638375,
        -0.0718483683742141,
        0.4909514227532223,
        -2.6454134031927374,
        -0.0399590564140291,
        0.0486778649724968,
        -0.0551575051990853,
        0.0354501802049087,
        -1.1130397141873534,
        0.0067890113155477,
        -0.0886810139593130,
        -0.0408207136116175,
        0.0427680029698811,
        -4.3436374104751501,
        -0.0049188492004049,
        0.1142109377736169,
        10.0527321757776047,
        0.0436512393438736,
        -154.12795711273924,
        422.12780928288305,
        -0.2105876149500405,
        -0.2126005780977984
    };

    auto f_mb = vm::vehicle_dynamics_mb(x_mb, u, p);

    // Python used default assertAlmostEqual (places=7).
    // That corresponds roughly to ~1e-7 absolute tolerance.
    check_vec(f_mb, f_mb_gt, 1e-7, "vehicle_dynamics_mb");
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------

int main()
{
    try {
        // load parameters (vehicle 2 is used in all but kst)
        auto p2 = vm::parameters_vehicle2();
        auto p4 = vm::parameters_vehicle4();

        // input
        const double g = 9.81;
        const double v_delta = 0.15;
        const double acc = 0.63 * g;

        std::vector<double> u{ v_delta, acc };

        test_ks(p2, u);
        test_kst(u);         // uses vehicle 4 internally
        test_st(p2, u);
        test_std(p2, u);
        test_mb(p2, u);

        std::cout << "All derivative tests PASSED." << std::endl;
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Exception in test_derivatives: " << e.what() << std::endl;
        return 1;
    }
}
