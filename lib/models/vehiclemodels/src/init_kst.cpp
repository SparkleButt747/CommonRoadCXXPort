#include "models/vehiclemodels/init_kst.hpp"

#include <algorithm>

namespace velox::models {

std::vector<double> init_kst(const std::vector<double>& init_state,
                             double alpha0)
{
    // Generate a safe 5-state base, padding or truncating as needed before
    // appending the hitch angle.
    constexpr std::size_t kBaseSize = 5;
    std::vector<double>   state(kBaseSize, 0.0);
    const auto            copy = std::min(init_state.size(), state.size());
    std::copy_n(init_state.begin(), copy, state.begin());

    state.push_back(alpha0); // hitch angle

    return state;
}

}  // namespace velox::models
