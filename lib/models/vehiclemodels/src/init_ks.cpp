#include "models/vehiclemodels/init_ks.hpp"

#include <algorithm>

namespace velox::models {

std::vector<double> init_ks(const std::vector<double>& init_state)
{
    // Always return a well-defined 5-state vector for the kinematic models.
    // If the caller provides fewer entries (or none), pad with zeros; if more,
    // truncate to the expected length to avoid out-of-bounds access.
    constexpr std::size_t kStateSize = 5;
    std::vector<double>   state(kStateSize, 0.0);
    const auto            copy = std::min(init_state.size(), state.size());
    std::copy_n(init_state.begin(), copy, state.begin());

    return state;
}

}  // namespace velox::models
