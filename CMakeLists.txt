cmake_minimum_required(VERSION 3.20)

project(velox
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
# Velox core library
# ------------------------------------------------------------

set(VELOX_LIB_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(VELOX_PARAM_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/parameters)
set(VELOX_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/config)

add_library(velox STATIC
    # controllers
    ${VELOX_LIB_DIR}/controllers/steering_controller.cpp
    ${VELOX_LIB_DIR}/controllers/longitudinal/aero.cpp
    ${VELOX_LIB_DIR}/controllers/longitudinal/brake.cpp
    ${VELOX_LIB_DIR}/controllers/longitudinal/final_accel_controller.cpp
    ${VELOX_LIB_DIR}/controllers/longitudinal/powertrain.cpp
    ${VELOX_LIB_DIR}/controllers/longitudinal/rolling_resistance.cpp

    # IO
    ${VELOX_LIB_DIR}/io/config_manager.cpp

    # models and utilities
    ${VELOX_LIB_DIR}/models/tire_model.cpp
    ${VELOX_LIB_DIR}/models/vehicle_dynamics_ks_cog.cpp

    ${VELOX_LIB_DIR}/models/vehiclemodels/src/vehicle_dynamics_ks.cpp
    ${VELOX_LIB_DIR}/models/vehiclemodels/src/vehicle_dynamics_kst.cpp
    ${VELOX_LIB_DIR}/models/vehiclemodels/src/vehicle_dynamics_linearized.cpp
    ${VELOX_LIB_DIR}/models/vehiclemodels/src/vehicle_dynamics_mb.cpp
    ${VELOX_LIB_DIR}/models/vehiclemodels/src/vehicle_dynamics_st.cpp
    ${VELOX_LIB_DIR}/models/vehiclemodels/src/vehicle_dynamics_std.cpp

    ${VELOX_LIB_DIR}/models/vehiclemodels/src/init_ks.cpp
    ${VELOX_LIB_DIR}/models/vehiclemodels/src/init_kst.cpp
    ${VELOX_LIB_DIR}/models/vehiclemodels/src/init_st.cpp
    ${VELOX_LIB_DIR}/models/vehiclemodels/src/init_mb.cpp
    ${VELOX_LIB_DIR}/models/vehiclemodels/src/init_std.cpp

    # simulation
    ${VELOX_LIB_DIR}/simulation/model_timing.cpp
    ${VELOX_LIB_DIR}/simulation/low_speed_safety.cpp
    ${VELOX_LIB_DIR}/simulation/simulation_daemon.cpp
    ${VELOX_LIB_DIR}/simulation/vehicle_simulator.cpp

    # telemetry
    ${VELOX_LIB_DIR}/telemetry/telemetry.cpp

    # parameter loader
    ${VELOX_PARAM_DIR}/vehicle_parameters.cpp
)

target_include_directories(velox
    PUBLIC
        $<BUILD_INTERFACE:${VELOX_LIB_DIR}>
        $<BUILD_INTERFACE:${VELOX_PARAM_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/velox>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_FULL_DATAROOTDIR}/velox/parameters>
)

target_compile_definitions(velox
    PUBLIC
        VELOX_PARAM_ROOT="${VELOX_PARAM_DIR}"
)

target_link_libraries(velox
    PUBLIC
        yaml-cpp::yaml-cpp
)

# ------------------------------------------------------------
# Tests
# ------------------------------------------------------------
option(BUILD_COMMONROAD_TESTS "Build unit and integration tests" OFF)

if (BUILD_COMMONROAD_TESTS)
    enable_testing()

    set(TEST_SOURCES
        tests/test_config_manager.cpp
        tests/test_longitudinal.cpp
        tests/test_low_speed_safety.cpp
        tests/test_telemetry_population.cpp
        tests/test_simulation_daemon.cpp
    )

    foreach(source ${TEST_SOURCES})
        get_filename_component(test_name ${source} NAME_WE)
        add_executable(${test_name} ${source})
        target_link_libraries(${test_name}
            PRIVATE
                velox
        )
        add_test(NAME ${test_name}
            COMMAND ${test_name}
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    endforeach()
endif()

install(TARGETS velox
    EXPORT veloxTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY ${VELOX_LIB_DIR}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/velox
    FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY ${VELOX_PARAM_DIR}/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/velox/parameters
)

install(DIRECTORY ${VELOX_CONFIG_DIR}/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/velox/config
)

install(EXPORT veloxTargets
    NAMESPACE velox::
    DESTINATION ${CMAKE_INSTALL_DATADIR}/cmake/velox
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
            velox
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

# Default OFF; enable with: cmake -DBUILD_VELOX_TESTS=ON ..
option(BUILD_VELOX_TESTS "Build Velox test executables" OFF)

enable_testing()

if(BUILD_VELOX_TESTS)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)

    add_executable(test_header_modules
        tests/test_header_modules.cpp
    )
    target_include_directories(test_header_modules
        PRIVATE
            ${VELOX_LIB_DIR}
    )
    add_test(NAME test_header_modules COMMAND test_header_modules)

    add_executable(test_derivatives
        tests/test_derivatives.cpp
    )
    target_link_libraries(test_derivatives
        PRIVATE
            velox
    )
    add_test(NAME test_derivatives COMMAND test_derivatives)

    add_executable(test_zeroInitialVelocity
        tests/test_zeroInitialVelocity.cpp
    )
    target_link_libraries(test_zeroInitialVelocity
        PRIVATE
            velox
    )
    add_test(NAME test_zeroInitialVelocity COMMAND test_zeroInitialVelocity)

    add_executable(test_vehicle
        tests/test_vehicle.cpp
    )
    target_link_libraries(test_vehicle
        PRIVATE
            velox
    )
    add_test(NAME test_vehicle COMMAND test_vehicle)

    add_executable(test_steering_controller
        tests/test_steering_controller.cpp
    )
    target_link_libraries(test_steering_controller
        PRIVATE
            velox
    )
    add_test(NAME test_steering_controller COMMAND test_steering_controller)

    add_executable(test_longitudinal
        tests/test_longitudinal.cpp
    )
    target_link_libraries(test_longitudinal
        PRIVATE
            velox
    )
    add_test(NAME test_longitudinal COMMAND test_longitudinal)

    add_executable(test_low_speed_safety
        tests/test_low_speed_safety.cpp
    )
    target_link_libraries(test_low_speed_safety
        PRIVATE
            velox
    )
    add_test(NAME test_low_speed_safety COMMAND test_low_speed_safety)

    add_executable(test_timestep_bounds
        tests/test_timestep_bounds.cpp
    )
    target_link_libraries(test_timestep_bounds
        PRIVATE
            velox
    )
    add_test(NAME test_timestep_bounds COMMAND test_timestep_bounds)

    add_executable(test_config_manager
        tests/test_config_manager.cpp
    )
    target_link_libraries(test_config_manager
        PRIVATE
            velox
    )
    add_test(NAME test_config_manager COMMAND test_config_manager)

    add_executable(scenario_simulator
        tests/scenario_simulator.cpp
    )
    target_link_libraries(scenario_simulator
        PRIVATE
            velox
    )

    add_test(NAME scenario_consistency
        COMMAND Python3::Interpreter
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_scenario_consistency.py
            --build-dir ${CMAKE_CURRENT_BINARY_DIR}
    )
endif()
