#include <cassert>
#include <vector>

#include "models/vehiclemodels/init_ks.hpp"
#include "models/vehiclemodels/init_kst.hpp"

namespace vm = velox::models;

int main()
{
    // KS: empty input pads to zeros.
    auto ks_default = vm::init_ks({});
    assert(ks_default.size() == 5);
    for (double v : ks_default) {
        assert(v == 0.0);
    }

    // KS: partial input is preserved and padded.
    std::vector<double> ks_partial{1.0, 2.0, 0.3};
    auto ks_padded = vm::init_ks(ks_partial);
    assert(ks_padded.size() == 5);
    assert(ks_padded[0] == 1.0);
    assert(ks_padded[1] == 2.0);
    assert(ks_padded[2] == 0.3);
    assert(ks_padded[3] == 0.0);
    assert(ks_padded[4] == 0.0);

    // KST: empty base plus provided hitch angle.
    const double alpha0 = 0.15;
    auto kst_default    = vm::init_kst({}, alpha0);
    assert(kst_default.size() == 6);
    for (std::size_t i = 0; i < 5; ++i) {
        assert(kst_default[i] == 0.0);
    }
    assert(kst_default[5] == alpha0);

    // KST: supplied base state is preserved, hitch angle forwarded.
    std::vector<double> kst_state{4.0, -1.0, 0.25, 3.0, 0.1, 0.33};
    auto kst_full = vm::init_kst(kst_state, kst_state.back());
    assert(kst_full.size() == 6);
    for (std::size_t i = 0; i < 5; ++i) {
        assert(kst_full[i] == kst_state[i]);
    }
    assert(kst_full[5] == kst_state.back());

    return 0;
}
