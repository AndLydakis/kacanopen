//
// Created by lydakis on 28/08/18.
//

#include <csignal>
#include <cstdlib>
#include "CanOpenMotor.h"

void testPositionAbsolute(CanOpenMotor *motor, int target, int threshold) {
    std::cout << "Test sending positive absolute target position " << target << " with threshold: " << threshold
              << std::endl;
    motor->readCurrentPosition();
    motor->readCurrent_position_internal();
    int32_t starting_position = motor->getCurrentPosition();
    int32_t starting_position_internal = motor->getCurrentPositionInternal();
    std::cout << "Starting position :" << starting_position << std::endl;
    std::cout << "Starting position internal :" << starting_position_internal << std::endl;
    motor->setTargetPosition(starting_position + target);

    std::cout << "Test reading target position\n";
    int32_t target_position = motor->getTargetPosition();
    assert(target_position == starting_position + target);
    std::cout << "Target position: " << target_position << std::endl;
    std::cout << "Sleeping to allow motor to move\n";
    usleep(3000000);
    assert(abs(motor->getCurrentPosition() - starting_position) <= (abs(target) + threshold));
}

void testPositionRelative(CanOpenMotor *motor, int target, int threshold) {
    std::cout << "Test sending positive relative target position " << target << " with threshold: " << threshold
              << std::endl;
    motor->readCurrentPosition();
    motor->readCurrent_position_internal();
    int32_t starting_position = motor->getCurrentPosition();
    int32_t starting_position_internal = motor->getCurrentPositionInternal();
    std::cout << "Starting position :" << starting_position << std::endl;
    std::cout << "Starting position internal :" << starting_position_internal << std::endl;
    motor->setTargetPositionRelative(target);

    std::cout << "Test reading target position\n";
    int32_t target_position = motor->getTargetPosition();
    assert(target_position == target);
    std::cout << "Target position: " << target_position << std::endl;
    std::cout << "Sleeping to allow motor to move\n";
    usleep(3000000);
    std::cout << abs(motor->getCurrentPosition() - starting_position) << std::endl;
    assert(abs(motor->getCurrentPosition() - starting_position) <= (abs(target) + threshold));
}

void testSwitchToVelocityMode(CanOpenMotor *motor) {
    std::cout << "Test switching to velocity mode\n";
    motor->setModeOfOperation(CanOpenMotor::VELOCITY);
    assert(motor->getModeOfOperation() == CanOpenMotor::VELOCITY);
}

void testSwitchToPositionMode(CanOpenMotor *motor) {
    std::cout << "Test switching to profile position mode\n";
    motor->setModeOfOperation(CanOpenMotor::PROFILE_POSITION);
    assert(motor->getModeOfOperation() == CanOpenMotor::PROFILE_POSITION);
}

void testVelocity(CanOpenMotor *motor, int velocity) {
    std::cout << "Test sending and reading target velocity\n";
    motor->setTargetVelocity(velocity);
    usleep(5000000);
    motor->readTargetVelocity();
    std::cout << "Target velocity " << motor->getTargetVelocity() << std::endl;
    assert(motor->getTargetVelocity() == 3000);
    std::cout << "Test stopping the motor\n";
    motor->setTargetVelocity(0);
    usleep(2000000);
    assert(motor->getCurrentVelocity() == 0);
}

void
testSetVelocityAcceleration(CanOpenMotor *motor, uint32_t dspeed = 20000, uint16_t dtime = 1) {
    std::cout << "Test setting velocity acceleration/deceleration\n";
    try {
        CanOpenMotor::velocity_accel va_send = {dspeed, dtime};
        motor->setVelocityAcceleration(0, va_send);
        motor->setVelocityAcceleration(1, va_send);
    } catch (kaco::canopen_error error) {
        std::cout << "Error while reading velocity acceleration/deceleration: " << error.what();
    }
}

void
testReadVelocityAcceleration(CanOpenMotor *motor, uint32_t dspeed = 20000, uint16_t dtime = 1) {
    std::cout << "Test reading velocity acceleration/deceleration \n";
    try {
        CanOpenMotor::velocity_accel va = motor->getVelocityAcceleration(0);
        std::cout << "Velocity Acceleration: " << va.delta_speed.value << "/" << va.delta_time.value << std::endl;
        va = motor->getVelocityAcceleration(1);
        std::cout << "Velocity Deceleration: " << va.delta_speed.value << "/" << va.delta_time.value << std::endl;
        assert(va.delta_speed.value == dspeed);
        assert(va.delta_time.value == dtime);
    } catch (kaco::canopen_error error) {
        std::cout << "Error while reading velocity acceleration/deceleration: " << error.what();
    }
}

void testReadMinMaxVelocity(CanOpenMotor *motor, uint32_t min = 60, uint32_t max = 25000) {
    std::cout << "Test reading min/max velocity and comparing to: " << min << "/" << max << std::endl;
    try {
        CanOpenMotor::min_max_velocity mmv = motor->getMinMaxVelocity();
        std::cout << "Velocity min/max = " << mmv.min_velocity.value << "/" << mmv.max_velocity.value << std::endl;
        assert(mmv.min_velocity.value == min);
        assert(mmv.max_velocity.value == max);
    } catch (kaco::canopen_error error) {
        std::cout << "Error while reading min/max velocity: " << error.what();
    }
}

void testSetMinMaxVelocity(CanOpenMotor *motor, uint32_t min_vel, uint32_t max_vel) {
    std::cout << "Test setting min/max velocity to: " << min_vel << "/" << max_vel << std::endl;
    try {
        motor->setMinMaxVelocity(min_vel, max_vel);
    } catch (kaco::canopen_error &error) {
        std::cout << "Error while reading velocity acceleration/deceleration: " << error.what();
    }
}

void testSetQuickStopRamp(CanOpenMotor *motor, uint32_t dspeed = 50000, uint16_t dtime = 1) {
    std::cout << "Test setting quick stop ramp to: " << dspeed << "/" << dtime << std::endl;
    try {
        motor->setQuickStopRamp(dspeed, dtime);
    } catch (kaco::canopen_error &error) {
        std::cout << "Error while setting quick stop ramp: " << error.what();
    }
}

void testReadQuickStopRamp(CanOpenMotor *motor, uint32_t dspeed = 50000, uint16_t dtime = 1) {
    std::cout << "Test reading quick stop ramp and comparing to: " << dspeed << "/" << dtime << std::endl;
    try {
        CanOpenMotor::velocity_accel mmv = motor->getQuickStopRamp();
        std::cout << "Velocity min/max = " << mmv.delta_speed.value << "/" << mmv.delta_time.value << std::endl;
        assert(mmv.delta_speed.value == dspeed);
        assert(mmv.delta_time.value == dtime);
    } catch (kaco::canopen_error &error) {
        std::cout << "Error while reading quick stop ramp: " << error.what();
    }
}

void testReadDimensionFactor(CanOpenMotor *motor, uint32_t nominator = 60, uint32_t denominator = 2000) {
    std::cout << "Test reading dimension factor and comparing to " << nominator << "/" << denominator << std::endl;
    try {
        CanOpenMotor::fraction_32_32 dim_factor = motor->getDimensionFactor();
        std::cout << "Dimension factor = " << dim_factor.nominator.value << "/" << dim_factor.denominator.value
                  << std::endl;
        assert(dim_factor.nominator.value == nominator);
        assert(dim_factor.denominator.value == denominator);
    } catch (kaco::canopen_error &error) {
        std::cout << "Error while reading dimension factor: " << error.what();
    }
}

void testSetDimensionFactor(CanOpenMotor *motor, uint32_t nominator = 60, uint32_t denominator = 2000) {
    std::cout << "Test setting dimension factor to: " << nominator << "/" << denominator << std::endl;
    try {
        motor->setQuickStopRamp(nominator, denominator);
    } catch (kaco::canopen_error &error) {
        std::cout << "Error while setting dimension factor: " << error.what();
    }
}

void testReadProfileAcceleration(CanOpenMotor *motor, uint8_t direction, uint32_t accel_) {
    std::cout << "Test reading profile " << (direction == 0 ? "Acc" : "Dec") << "eleration and comparing to " << accel_
              << std::endl;
    try {
        uint32_t profile_accel = motor->getProfileAcceleration(direction);
        std::cout << "Profile " << (direction == 0 ? "Acc" : "Dec") << "eleration = " << profile_accel << std::endl;
        assert(profile_accel == accel_);
    } catch (kaco::canopen_error &error) {
        std::cout << "Error while reading profile acceleration: " << error.what();
    }
}

void testSetProfileAcceleration(CanOpenMotor *motor, uint8_t direction, uint32_t accel = 2000) {
    std::cout << "Test setting profile " << (direction == 0 ? "Acc" : "Dec") << "eleration to: " << accel << std::endl;
    try {
        motor->setProfileAcceleration(direction, accel);
    } catch (kaco::canopen_error &error) {
        std::cout << "Error while setting profile acceleration/deceleration: " << error.what();
    }
}

void testReadQuickStopDeceleration(CanOpenMotor *motor, uint32_t decel_) {
    std::cout << "Test reading profile quick stop deceleration and comparing to " << decel_
              << std::endl;
    try {
        uint32_t quick_stop_decel = motor->getQuickStopDeceleration();
        std::cout << "Profile quick stop deceleration: " << quick_stop_decel << std::endl;
        assert(quick_stop_decel == decel_);
    } catch (kaco::canopen_error &error) {
        std::cout << "Error while reading profile acceleration/deceleration: " << error.what();
    }
}

void testSetQuickStopDeceleration(CanOpenMotor *motor, uint32_t decel = 2000) {
    std::cout << "Test setting profile quick stop deceleration to: " << decel << std::endl;
    try {
        motor->setQuickStopDeceleration(decel);
    } catch (kaco::canopen_error &error) {
        std::cout << "Error while setting profile acceleration/deceleration: " << error.what();
    }
}

void testReadMaxAcceleration(CanOpenMotor *motor, uint8_t direction, uint32_t accel_) {
    std::cout << "Test reading max " << (direction == 0 ? "Acc" : "Dec") << "eleration and comparing to " << accel_
              << std::endl;
    try {
        uint32_t profile_accel = motor->getMaxAcceleration(direction);
        std::cout << "Max " << (direction == 0 ? "Acc" : "Dec") << "eleration = " << profile_accel << std::endl;
        assert(profile_accel == accel_);
    } catch (kaco::canopen_error &error) {
        std::cout << "Error while reading max acceleration/deceleration: " << error.what();
    }
}

void testSetMaxAcceleration(CanOpenMotor *motor, uint8_t direction, uint32_t accel = 2000) {
    std::cout << "Test setting max " << (direction == 0 ? "Acc" : "Dec") << "eleration to: " << accel << std::endl;
    try {
        motor->setMaxAcceleration(direction, accel);
    } catch (kaco::canopen_error &error) {
        std::cout << "Error while setting maxacceleration/deceleration: " << error.what();
    }
}


int main(int argc, char **argv) {
    kaco::Master master;
    ros::init(argc, argv, "can_test_node");
    // Set the name of your CAN bus. "slcan0" is a common bus name
    // for the first SocketCAN device on a Linux system.
    const std::string busname = argv[1];
    std::cout << "Opening /dev/pcan" << busname << std::endl;
    // Set the baudrate of your CAN bus. Most drivers support the values
    // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
    const std::string baudrate = argv[2];
    std::cout << "With baudrate " << baudrate << std::endl;

    const size_t bus_devices = static_cast<const size_t>(std::stoi(argv[3]));

    std::cout << "Starting Master (involves starting Core)..." << std::endl;
    if (!master.start(busname, baudrate)) {
        std::cout << "Starting Master failed." << std::endl;
        return EXIT_FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const size_t num_devices = master.num_devices();

    std::cout << num_devices << " devices found on the bus\n";
    assert(num_devices == bus_devices);
    ros::NodeHandle nh;
    for (size_t i = 0; i < num_devices; ++i) {
        std::cout << "Testing motor " << i + 1 << std::endl;
        std::cout << "Test initializing the device\n";
        auto *motor = new CanOpenMotor(i + 1, std::stoi(baudrate), nh, master);
        assert(motor->getId() == i + 1);
        motor->print();
        std::cout << "***************************************************************************************\n";
        uint32_t max_allowed_acceleration = 25000000;
        testSetMaxAcceleration(motor, 1, max_allowed_acceleration);
        testReadMaxAcceleration(motor, 1, max_allowed_acceleration);
        testSetMaxAcceleration(motor, 0, max_allowed_acceleration);
        testReadMaxAcceleration(motor, 0, max_allowed_acceleration);

//        std::cout << "***************************************************************************************\n";
//        uint32_t quick_stop_deceleration = 100000;
//        testSetQuickStopDeceleration(motor, quick_stop_deceleration);
//        testReadQuickStopDeceleration(motor, quick_stop_deceleration);
//        std::cout << "***************************************************************************************\n";
//        uint32_t profile_acceleration = 20000;
//        testSetProfileAcceleration(motor, 0, profile_acceleration);
//        testReadProfileAcceleration(motor, 0, profile_acceleration);
//        testSetProfileAcceleration(motor, 1, profile_acceleration);
//        testReadProfileAcceleration(motor, 1, profile_acceleration);
//        std::cout << "***************************************************************************************\n";
//        uint32_t dim_factor_nom = 60;
//        uint32_t dim_factor_denom = 2000;
//        testSetDimensionFactor(motor, dim_factor_nom, dim_factor_denom);
//        testReadDimensionFactor(motor, dim_factor_nom, dim_factor_denom);
//        std::cout << "***************************************************************************************\n";
//        uint32_t qs_dspeed = 50000;
//        uint16_t qs_dtime = 1;
//        testSetQuickStopRamp(motor, qs_dspeed, qs_dtime);
//        testReadQuickStopRamp(motor, qs_dspeed, qs_dtime);
//        std::cout << "***************************************************************************************\n";
//        uint32_t min_vel = 60;
//        uint32_t max_vel = 25000;
//        testSetMinMaxVelocity(motor, min_vel, max_vel);
//        testReadMinMaxVelocity(motor);
//        std::cout << "***************************************************************************************\n";
//        uint32_t accel_dspeed = 20000;
//        uint32_t accel_dtime = 1;
//        testSetVelocityAcceleration(motor, accel_dspeed, accel_dtime);
//        testReadVelocityAcceleration(motor, accel_dspeed, accel_dtime);
//        std::cout << "***************************************************************************************\n";
//        testSwitchToPositionMode(motor);
//        std::cout << "***************************************************************************************\n";
//        int position_command = 1024;
//        testPositionRelative(motor, position_command, 10);
//        testPositionRelative(motor, -position_command, 10);
//        std::cout << "***************************************************************************************\n";
//        testPositionAbsolute(motor, position_command, 10);
//        testPositionAbsolute(motor, -position_command, 10);
//        std::cout << "***************************************************************************************\n";
//        testSwitchToVelocityMode(motor);
//        std::cout << "***************************************************************************************\n";
//        int velocity = 3000;
//        testVelocity(motor, velocity);
//        std::cout << "***************************************************************************************\n";
//        motor->calibrate();
//        std::cout << "***************************************************************************************\n";
//        motor->print();
//        std::cout << "***************************************************************************************\n";
        std::cout << "Completed tests for device " << i + 1 << std::endl;
        delete (motor);
    }
    return EXIT_SUCCESS;
}

