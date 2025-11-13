cmake_minimum_required(VERSION 3.20)

project(commonroad_cpp
    VERSION 0.1.0
    LANGUAGES CXX)

# ------------------------------------------------------------
# Global settings
# ------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (MSVC)
    add_compile_options(/W4 /permissive-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------
# External packages
# ------------------------------------------------------------

# SDL2 (installed via system / Homebrew / vcpkg, etc.)
find_package(SDL2 REQUIRED)

# yaml-cpp (for parameter loading)
find_package(yaml-cpp REQUIRED)

# ------------------------------------------------------------
# ImGui (vendored in third_party/imgui)
# ------------------------------------------------------------

set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui)

add_library(imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp

    # backends
    ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdlrenderer2.cpp
)

target_include_directories(imgui
    PUBLIC
        ${IMGUI_DIR}
        ${IMGUI_DIR}/backends
)

# ImGui backend depends on SDL2 headers
target_link_libraries(imgui
    PUBLIC
        SDL2::SDL2
)

# ------------------------------------------------------------
# CommonRoad core library
# ------------------------------------------------------------

# Source roots
set(COMMONROAD_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(COMMONROAD_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(COMMONROAD_PARAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/parameters)

add_library(commonroad STATIC
    # utils
    ${COMMONROAD_SRC_DIR}/utils/acceleration_constraints.cpp
    ${COMMONROAD_SRC_DIR}/utils/longitudinal_parameters.cpp
    ${COMMONROAD_SRC_DIR}/utils/steering_constraints.cpp
    ${COMMONROAD_SRC_DIR}/utils/steering_parameters.cpp
    ${COMMONROAD_SRC_DIR}/utils/tire_model.cpp
    ${COMMONROAD_SRC_DIR}/utils/tireParameters.cpp
    ${COMMONROAD_SRC_DIR}/utils/trailer_parameters.cpp
    ${COMMONROAD_SRC_DIR}/utils/unitConversion.cpp
    ${COMMONROAD_SRC_DIR}/utils/vehicle_dynamics_ks_cog.cpp

    # vehicle dynamics
    ${COMMONROAD_SRC_DIR}/vehiclemodels/vehicle_dynamics_ks.cpp
    ${COMMONROAD_SRC_DIR}/vehiclemodels/vehicle_dynamics_kst.cpp
    ${COMMONROAD_SRC_DIR}/vehiclemodels/vehicle_dynamics_linearized.cpp
    ${COMMONROAD_SRC_DIR}/vehiclemodels/vehicle_dynamics_mb.cpp
    ${COMMONROAD_SRC_DIR}/vehiclemodels/vehicle_dynamics_st.cpp
    ${COMMONROAD_SRC_DIR}/vehiclemodels/vehicle_dynamics_std.cpp

    # parameter loader
    ${COMMONROAD_PARAM_DIR}/vehicle_parameters.cpp
)

target_include_directories(commonroad
    PUBLIC
        ${COMMONROAD_INCLUDE_DIR}        # include/utils, include/vehiclemodels
        ${COMMONROAD_PARAM_DIR}          # vehicle_parameters.hpp, vehicle/parameters_vehicle*.hpp
)

# Define default root for parameter YAMLs as an absolute path
target_compile_definitions(commonroad
    PUBLIC
        COMMONROAD_PARAM_ROOT=\"${COMMONROAD_PARAM_DIR}\"
)

# Link yaml-cpp into the core library
target_link_libraries(commonroad
    PUBLIC
        yaml-cpp::yaml-cpp
)

# ------------------------------------------------------------
# App: SDL2 + ImGui viewer / keyboard controller
# ------------------------------------------------------------

add_executable(commonroad_app
    app/main.cpp
)

target_link_libraries(commonroad_app
    PRIVATE
        commonroad
        imgui
        SDL2::SDL2
        SDL2::SDL2main
)

# On macOS, make sure frameworks are handled correctly
if (APPLE)
    # Some SDL2 builds on macOS rely on frameworks instead of libs;
    # if needed you can add:
    # target_link_libraries(commonroad_app PRIVATE "-framework Cocoa" "-framework IOKit" "-framework CoreVideo")
endif()

# ------------------------------------------------------------
# Tests
# ------------------------------------------------------------

# Default OFF; you can enable with: cmake -DBUILD_COMMONROAD_TESTS=ON ..
option(BUILD_COMMONROAD_TESTS "Build CommonRoad test executables" OFF)

if(BUILD_COMMONROAD_TESTS)
    add_executable(test_derivatives
        tests/test_derivatives.cpp
    )
    target_link_libraries(test_derivatives
        PRIVATE
            commonroad
    )

    # we'll add test_zeroInitialVelocity once init_ks/init_st/init_mb are ported
    # add_executable(test_zeroInitialVelocity
    #     tests/test_zeroInitialVelocity.cpp
    # )
    # target_link_libraries(test_zeroInitialVelocity
    #     PRIVATE
    #         commonroad
    # )
endif()



