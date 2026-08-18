# Explicitly name it because otherwise Zephyr appends _ to the beginning because we're up a level on the directory
zephyr_library_named(core)
zephyr_library_sources(
    # Algo Section
    util/algorithms/general_functions.cpp

    # Comms Section
    util/communications/CANMsg.h
    util/communications/CANHandler.h
    util/communications/DJIRemote2.cpp
    util/communications/ui/ui_g.cpp
    util/communications/referee/ref_serial.cpp
    util/communications/referee/crc.cpp
    
    # Misc Util
    util/motor/DJIMotor.cpp
    util/peripherals/imu/ISM330.cpp
    util/peripherals/encoder/MA4.cpp

    # Subsystems
    subsystems/ChassisSubsystem.cpp
    subsystems/TurretSubsystem.cpp

)
zephyr_library_include_directories(
    .
    util/algorithms
    util/communications
    util/motor
    util/peripherals
    subsystems
)