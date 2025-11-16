#include "models/vehiclemodels/init_st.hpp"

namespace velox::models {

std::vector<double> init_st(const std::vector<double>& init_state)
{
    // Direct pass-through, as in the Python implementation.
    return init_state;
}

}  // namespace velox::models
