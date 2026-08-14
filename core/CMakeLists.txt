# Explicitly name it because otherwise Zephyr appends _ to the beginning because we're up a level on the directory
zephyr_library_named(core)
zephyr_library_sources(
    util/algorithms/general_functions.cpp
    util/communications/DJIRemote2.cpp
    util/communications/CANMsg.h
    util/communications/CANHandler.h
    util/motor/DJIMotor.cpp
    util/peripherals/imu/ISM330.cpp
)
zephyr_library_include_directories(
    .
    util/algorithms
    util/communications
    util/motor
    util/peripherals
    subsystems
)