cmake_minimum_required(VERSION 3.20)

project(commonroad_cpp
    VERSION 0.1.0
    LANGUAGES CXX)

include(GNUInstallDirs)

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

if (EXISTS "${IMGUI_DIR}/imgui.cpp")
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
endif()

# ------------------------------------------------------------
# CommonRoad core library
# ------------------------------------------------------------

# Source roots
set(COMMONROAD_SRC_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(COMMONROAD_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(COMMONROAD_PARAM_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/parameters)
set(COMMONROAD_CONFIG_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/config)

add_library(commonroad STATIC
    # utils
    ${COMMONROAD_SRC_DIR}/utils/acceleration_constraints.cpp
    ${COMMONROAD_SRC_DIR}/utils/longitudinal_parameters.cpp
    ${COMMONROAD_SRC_DIR}/utils/steering_controller.cpp
    ${COMMONROAD_SRC_DIR}/utils/steering_constraints.cpp
    ${COMMONROAD_SRC_DIR}/utils/steering_parameters.cpp
    ${COMMONROAD_SRC_DIR}/utils/tire_model.cpp
    ${COMMONROAD_SRC_DIR}/utils/tireParameters.cpp
    ${COMMONROAD_SRC_DIR}/utils/trailer_parameters.cpp
    ${COMMONROAD_SRC_DIR}/utils/unitConversion.cpp
    ${COMMONROAD_SRC_DIR}/utils/vehicle_dynamics_ks_cog.cpp

    # longitudinal sim
    ${COMMONROAD_SRC_DIR}/sim/actuators/aero.cpp
    ${COMMONROAD_SRC_DIR}/sim/actuators/brake.cpp
    ${COMMONROAD_SRC_DIR}/sim/actuators/powertrain.cpp
    ${COMMONROAD_SRC_DIR}/sim/actuators/rolling_resistance.cpp
    ${COMMONROAD_SRC_DIR}/sim/controllers/final_accel_controller.cpp
    ${COMMONROAD_SRC_DIR}/sim/longitudinal/config_loader.cpp
    ${COMMONROAD_SRC_DIR}/sim/low_speed_safety.cpp
    ${COMMONROAD_SRC_DIR}/sim/low_speed_safety_loader.cpp
    ${COMMONROAD_SRC_DIR}/sim/vehicle_simulator.cpp

    # vehicle dynamics
    ${COMMONROAD_SRC_DIR}/vehiclemodels/vehicle_dynamics_ks.cpp
    ${COMMONROAD_SRC_DIR}/vehiclemodels/vehicle_dynamics_kst.cpp
    ${COMMONROAD_SRC_DIR}/vehiclemodels/vehicle_dynamics_linearized.cpp
    ${COMMONROAD_SRC_DIR}/vehiclemodels/vehicle_dynamics_mb.cpp
    ${COMMONROAD_SRC_DIR}/vehiclemodels/vehicle_dynamics_st.cpp
    ${COMMONROAD_SRC_DIR}/vehiclemodels/vehicle_dynamics_std.cpp

    # init functions (ports of Python init_*)
    ${COMMONROAD_SRC_DIR}/vehiclemodels/init_ks.cpp
    ${COMMONROAD_SRC_DIR}/vehiclemodels/init_kst.cpp
    ${COMMONROAD_SRC_DIR}/vehiclemodels/init_st.cpp
    ${COMMONROAD_SRC_DIR}/vehiclemodels/init_mb.cpp
    ${COMMONROAD_SRC_DIR}/vehiclemodels/init_std.cpp

    # parameter loader
    ${COMMONROAD_PARAM_DIR}/vehicle_parameters.cpp
)

target_include_directories(commonroad
    PUBLIC
        $<BUILD_INTERFACE:${COMMONROAD_INCLUDE_DIR}>  # include/utils, include/vehiclemodels, include/vehicle, include/sim
        $<BUILD_INTERFACE:${COMMONROAD_PARAM_DIR}>    # vehicle_parameters.hpp, vehicle/parameters_vehicle*.hpp
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_FULL_DATAROOTDIR}/commonroad/parameters>
)

# Define default root for parameter YAMLs as an absolute path
target_compile_definitions(commonroad
    PUBLIC
        COMMONROAD_PARAM_ROOT="$<BUILD_INTERFACE:${COMMONROAD_PARAM_DIR}>$<INSTALL_INTERFACE:${CMAKE_INSTALL_FULL_DATAROOTDIR}/commonroad/parameters>"
)

# Link yaml-cpp into the core library
target_link_libraries(commonroad
    PUBLIC
        yaml-cpp::yaml-cpp
)

install(TARGETS commonroad
    EXPORT commonroadTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY ${COMMONROAD_INCLUDE_DIR}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${COMMONROAD_PARAM_DIR}/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/commonroad/parameters
)

install(DIRECTORY ${COMMONROAD_CONFIG_DIR}/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/commonroad/config
)

install(EXPORT commonroadTargets
    NAMESPACE commonroad::
    DESTINATION ${CMAKE_INSTALL_DATADIR}/cmake/commonroad
)

# ------------------------------------------------------------
# App: SDL2 + ImGui viewer / keyboard controller
# ------------------------------------------------------------

if (TARGET imgui)
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

    if (APPLE)
        # If your SDL2 package is framework-based and complains, you can uncomment:
        # target_link_libraries(commonroad_app PRIVATE "-framework Cocoa" "-framework IOKit" "-framework CoreVideo")
    endif()
else()
    message(STATUS "ImGui sources not present; skipping commonroad_app build")
endif()

# ------------------------------------------------------------
# Tests
# ------------------------------------------------------------

# Default OFF; enable with: cmake -DBUILD_COMMONROAD_TESTS=ON ..
option(BUILD_COMMONROAD_TESTS "Build CommonRoad test executables" OFF)

enable_testing()

if(BUILD_COMMONROAD_TESTS)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)

    add_executable(test_derivatives
        tests/test_derivatives.cpp
    )
    target_link_libraries(test_derivatives
        PRIVATE
            commonroad
    )
    add_test(NAME test_derivatives COMMAND test_derivatives)

    add_executable(test_zeroInitialVelocity
        tests/test_zeroInitialVelocity.cpp
    )
    target_link_libraries(test_zeroInitialVelocity
        PRIVATE
            commonroad
    )
    add_test(NAME test_zeroInitialVelocity COMMAND test_zeroInitialVelocity)

    add_executable(test_vehicle
        tests/test_vehicle.cpp
    )
    target_link_libraries(test_vehicle
        PRIVATE
            commonroad
    )
    add_test(NAME test_vehicle COMMAND test_vehicle)

    add_executable(test_steering_controller
        tests/test_steering_controller.cpp
    )
    target_link_libraries(test_steering_controller
        PRIVATE
            commonroad
    )
    add_test(NAME test_steering_controller COMMAND test_steering_controller)

    add_executable(test_longitudinal
        tests/test_longitudinal.cpp
    )
    target_link_libraries(test_longitudinal
        PRIVATE
            commonroad
    )
    add_test(NAME test_longitudinal COMMAND test_longitudinal)

    add_executable(test_low_speed_safety
        tests/test_low_speed_safety.cpp
    )
    target_link_libraries(test_low_speed_safety
        PRIVATE
            commonroad
    )
    add_test(NAME test_low_speed_safety COMMAND test_low_speed_safety)

    add_executable(test_timestep_bounds
        tests/test_timestep_bounds.cpp
    )
    target_link_libraries(test_timestep_bounds
        PRIVATE
            commonroad
    )
    add_test(NAME test_timestep_bounds COMMAND test_timestep_bounds)

    add_executable(scenario_simulator
        tests/scenario_simulator.cpp
    )
    target_link_libraries(scenario_simulator
        PRIVATE
            commonroad
    )

    add_test(NAME scenario_consistency
        COMMAND Python3::Interpreter
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_scenario_consistency.py
            --build-dir ${CMAKE_CURRENT_BINARY_DIR}
    )
endif()
