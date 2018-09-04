//
// Created by lydakis on 28/08/18.
//
#include <CanOpenMotor.h>

#include "CanOpenMotor.h"

CanOpenMotor::CanOpenMotor(size_t id_, int baudrate_, ros::NodeHandle &nh_, kaco::Master &master_) : CanOpenDevice(id_,
                                                                                                                   baudrate_,
                                                                                                                   nh_,
                                                                                                                   master_) {}

CanOpenMotor::CanOpenMotor(size_t id_, int baudrate_, kaco::Master &master_) : CanOpenDevice(id_,
                                                                                             baudrate_,
                                                                                             master_) {}

int32_t CanOpenMotor::getCurrentPosition() {
    readCurrentPosition();
    return current_position;
}

void CanOpenMotor::setCurrentPosition(int32_t current_position) {
    CanOpenMotor::current_position = current_position;
}

int32_t CanOpenMotor::getCurrentPositionInternal() {
    readCurrent_position_internal();
    return current_position_internal;
}

void CanOpenMotor::setCurrentPositionInternal(int32_t current_position_internal) {
    CanOpenMotor::current_position_internal = current_position_internal;
}

int32_t CanOpenMotor::getTargetPosition() {
    readTargetPosition();
    return target_position;
}

void CanOpenMotor::setTargetPosition(int32_t target_position) {
    device.execute("set_target_position", target_position);
    readTargetPosition();
}

int CanOpenMotor::getTargetVelocity() const {
    return target_velocity;
}

void CanOpenMotor::setTargetVelocity(int target_velocity) {
    device.execute("set_target_velocity", (int16_t) target_velocity);
    readTargetVelocity();
}

int CanOpenMotor::getCurrentVelocity() {
    readCurrentVelocity();
    return current_velocity;
}

void CanOpenMotor::setCurrentVelocity(int current_velocity) {
    CanOpenMotor::current_velocity = static_cast<int16_t>(current_velocity);
}

int16_t CanOpenMotor::getStatus_word() const {
    return status_word;
}

void CanOpenMotor::setStatus_word(unsigned int status_word) {
    CanOpenMotor::status_word = static_cast<int16_t>(status_word);
}

void CanOpenMotor::readCurrentPosition() {
    try { current_position = int32_t(get_attribute("position_actual_value")); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not read position (SDO 0x6064/0): " << error.what() << std::endl;
    }
}

void CanOpenMotor::readCurrent_position_internal() {
    try { current_position_internal = int32_t(get_attribute("position_actual_value*")); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not read internal position (SDO 0x6063/0): " << error.what() << std::endl;
    }
}

void CanOpenMotor::readTargetPosition() {
    try { target_position = int32_t(get_attribute("target_position")); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not read target position (SDO 0x607a): " << error.what() << std::endl;
    }

}

void CanOpenMotor::readTargetVelocity() {
    try { target_velocity = int16_t(get_attribute("vl_target_velocity")); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not read target velocity (SDO 0x6042): " << error.what() << std::endl;
    }
}

void CanOpenMotor::readCurrentVelocity() {
    try { current_velocity = int16_t(get_attribute("velocity_actual_value")); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not read current velocity (SDO 0x606C/0)" << error.what() << std::endl;
    }
}

void CanOpenMotor::readStatus_word() {
    try { status_word = int16_t(get_attribute("status_word")); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not read status word (SDO 0x6041)" << error.what() << std::endl;
    }
}

void CanOpenMotor::setModeOfOperation(CanOpenMotor::MotorMode mode_) {
//    std::cout << "Switching mode to " << mode_ << " " << modes[mode_] << " : " << device.get_constant(modes[mode_])
//              << std::endl;
    try {
        DisableOperation();
        device.set_entry("modes_of_operation", device.get_constant(modes[mode_]));
        EnableOperation();
        readModeOfOperation();
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set mode of operation\n";
    }

}

CanOpenMotor::MotorMode CanOpenMotor::getModeOfOperation() {
    try {
        readModeOfOperation();
        return mode;
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not get mode of operation: " << error.what() << std::endl;
        return UNDEFINED;
    }
}

void CanOpenMotor::readModeOfOperation() {
    mode = static_cast<MotorMode>(int8_t(device.get_entry("modes_of_operation")));
}

bool CanOpenMotor::enableVelocityMode() {
    try {
        setModeOfOperation(VELOCITY);
        readModeOfOperation();
        return mode == VELOCITY;
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not enable velocity mode: " << error.what() << std::endl;
        return false;
    }
}

bool CanOpenMotor::enableProfilePositionMode() {
    try {
        setModeOfOperation(PROFILE_POSITION);
        readModeOfOperation();
        return mode == PROFILE_POSITION;
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not enable profile position mode: " << error.what() << std::endl;
        return false;
    }
}

bool CanOpenMotor::enableProfileTorqueMode() {
    try {
        setModeOfOperation(PROFILE_TORQUE);
        readModeOfOperation();
        return mode == PROFILE_TORQUE;
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not enable profile torque mode: " << error.what() << std::endl;
        return false;
    }
}

void CanOpenMotor::setTargetPositionRelative(int32_t target_position_) {
    device.execute("set_target_position_relative", target_position_);
    readTargetPosition();
}

void CanOpenMotor::calibrate() {
    std::cout << "Calibrating motor\n";
    int sum = 0;
    setModeOfOperation(PROFILE_POSITION);
    assert(getModeOfOperation() == PROFILE_POSITION);
    readCurrent_position_internal();
    readCurrentPosition();
    try {
        for (size_t i = 0; i < DEFAULT_CALIB_RUNS; i++) {
            long starting_position = getCurrentPosition();
            this->setTargetPositionRelative(int32_t(1024));
            usleep(2000000);
            long diff = getCurrentPosition() - starting_position;
            std::cout << "Calibration run " << i + 1 << " starting in position " << starting_position
                      << " and ending in position " << getCurrentPosition() << std::endl;
            sum += (diff);
        }

    } catch (const kaco::canopen_error &error) {
        std::cout << "Could not finish calibration\n";
        std::cout << error.what() << std::endl;
        steps_per_rotation = static_cast<size_t>(-1);
    }
    steps_per_rotation = static_cast<size_t>(sum / DEFAULT_CALIB_RUNS);
}

void CanOpenMotor::print() {
    std::cout << "############ Nanotec CanOpen Motor ############\n";
    std::cout << "Current Position: " << current_position << std::endl;
    std::cout << "Internal Current Position: " << current_position_internal << std::endl;
    std::cout << "Target Position: " << target_position << std::endl;
    std::cout << "Current Velocity: " << current_velocity << std::endl;
    std::cout << "Target Velocity:  " << target_velocity << std::endl;
    std::cout << "Operating Mode: " << mode << " (" << modes[mode] << ")" << std::endl;
    std::cout << "Steps per rotation: " << steps_per_rotation << std::endl;
    std::cout << "Available modes of operation: " << device.get_entry("modes_of_operation_display") << std::endl;
    std::cout << "###############################################\n";
}


uint16_t CanOpenMotor::getVelocityWindow() {
    try {
        return device.get_entry("velocity_window");
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read velocity_window (SDO 0x606D/0): " << error.what() << std::endl;
        return static_cast<uint16_t>(-1);
    }
}

void CanOpenMotor::setVelocityWindow(int16_t velocity_window) {
    try {
        device.set_entry("vl_velocity_window", velocity_window);
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set vl_velocity_window(SDO 0x606D/0): " << error.what() << std::endl;
    }
}

/***********************************************************************************/
CanOpenMotor::velocity_accel CanOpenMotor::getVelocityAcceleration(uint8_t direction) {
    assert((direction == 0) || (direction == 1));
    velocity_accel va = {0, 0};
    try {
        std::vector<uint8_t> delta_speed = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6048 + direction,
                                                                  0x0001);
        memcpy(&va.delta_speed.value, &delta_speed[0], std::min(delta_speed.size(), sizeof(uint32_t)));

        std::vector<uint8_t> delta_time = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6048 + direction,
                                                                 0x0002);
        memcpy(&va.delta_time.value, &delta_time[0], std::min(delta_time.size(), sizeof(uint16_t)));
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read velocity acceleration/deceleration (SDO 0x6048/01-02):  " << error.what()
                  << std::endl;
    }
    return va;
}

void CanOpenMotor::setVelocityAcceleration(uint8_t direction, CanOpenMotor::velocity_accel new_val) {
    assert((direction == 0) || (direction == 1));
    std::vector<uint8_t> dspeed(new_val.delta_speed.array, new_val.delta_speed.array +
                                                           sizeof new_val.delta_speed.array /
                                                           sizeof new_val.delta_speed.array[0]);

    std::vector<uint8_t> dtime(new_val.delta_time.array, new_val.delta_time.array +
                                                         sizeof new_val.delta_time.array /
                                                         sizeof new_val.delta_time.array[0]);
    try {
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x6048 + direction, 0x0001,
                                 static_cast<uint32_t>(dspeed.size()), dspeed);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x6048 + direction, 0x0002,
                                 static_cast<uint32_t>(dtime.size()), dtime);
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set velocity acceleration/deceleration (SDO 0x6048-9/01-02):  " << error.what()
                  << std::endl;
    }
}

/***********************************************************************************/
CanOpenMotor::min_max_velocity CanOpenMotor::getMinMaxVelocity() {
    min_max_velocity mmv{0, 0};
    try {
        std::vector<uint8_t> min_vel = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6046, 0x0001);
        memcpy(&mmv.min_velocity.value, &min_vel[0], std::min(min_vel.size(), sizeof(uint32_t)));

        std::vector<uint8_t> max_vel = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6046, 0x0002);
        memcpy(&mmv.max_velocity.value, &max_vel[0], std::min(max_vel.size(), sizeof(uint16_t)));
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read min/max velocity limits (SDO 0x6046/01-02): " << error.what() << std::endl;
    }
    return mmv;
}

void CanOpenMotor::setMinMaxVelocity(uint32_t min, uint32_t max) {
    min_max_velocity mmv{min, max};
    try {
        std::vector<uint8_t> min_vel(mmv.min_velocity.array, mmv.min_velocity.array +
                                                             sizeof mmv.min_velocity.array /
                                                             sizeof mmv.min_velocity.array[0]);

        std::vector<uint8_t> max_vel(mmv.max_velocity.array, mmv.max_velocity.array +
                                                             sizeof mmv.max_velocity.array /
                                                             sizeof mmv.max_velocity.array[0]);

        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x6046, 0x0001, static_cast<uint32_t>(min_vel.size()),
                                 min_vel);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x6046, 0x0002, static_cast<uint32_t>(max_vel.size()),
                                 max_vel);

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set min/max velocity limits (SDO 0x6046/01-02): " << error.what() << std::endl;
    }
}

/***********************************************************************************/
CanOpenMotor::velocity_accel CanOpenMotor::getQuickStopRamp() {
    velocity_accel qs = {0, 0};
    try {
        std::vector<uint8_t> delta_speed = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x604A, 0x0001);
        memcpy(&qs.delta_speed.value, &delta_speed[0], std::min(delta_speed.size(), sizeof(uint32_t)));

        std::vector<uint8_t> delta_time = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x604A, 0x0002);
        memcpy(&qs.delta_time.value, &delta_time[0], std::min(delta_time.size(), sizeof(uint16_t)));
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read velocity quick stop ramp (SDO 0x604A): " << error.what() << std::endl;
    }
    return qs;
}

void CanOpenMotor::setQuickStopRamp(uint32_t delta_speed, uint16_t delta_time) {
    velocity_accel quick_ramp{delta_speed, delta_time};
    setQuickStopRamp(quick_ramp);

}

void CanOpenMotor::setQuickStopRamp(CanOpenMotor::velocity_accel quick_ramp) {
    try {
        std::vector<uint8_t> dspeed(quick_ramp.delta_speed.array, quick_ramp.delta_speed.array +
                                                                  sizeof quick_ramp.delta_speed.array /
                                                                  sizeof quick_ramp.delta_speed.array[0]);

        std::vector<uint8_t> dtime(quick_ramp.delta_time.array, quick_ramp.delta_time.array +
                                                                sizeof quick_ramp.delta_time.array /
                                                                sizeof quick_ramp.delta_time.array[0]);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x604A, 0x0001, static_cast<uint32_t>(dspeed.size()),
                                 dspeed);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x604A, 0x0002, static_cast<uint32_t>(dtime.size()),
                                 dtime);

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set velocity quick stop ramp (SDO 0x604A): " << error.what() << std::endl;
    }
}

/***********************************************************************************/
CanOpenMotor::fraction_32_32 CanOpenMotor::getDimensionFactor() {
    fraction_32_32 qs = {0, 0};
    try {
        std::vector<uint8_t> delta_speed = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x604C, 0x0001);
        memcpy(&qs.nominator.value, &delta_speed[0], std::min(delta_speed.size(), sizeof(uint32_t)));

        std::vector<uint8_t> delta_time = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x604C, 0x0002);
        memcpy(&qs.denominator.value, &delta_time[0], std::min(delta_time.size(), sizeof(uint16_t)));
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read velocity quick stop ramp (SDO 0x604c): " << error.what() << std::endl;
    }
    return qs;
}

void CanOpenMotor::setDimensionFactor(uint32_t nominator, uint32_t denominator) {
    fraction_32_32 fraction(nominator, denominator);
    setDimensionFactor(fraction);
}

void CanOpenMotor::setDimensionFactor(CanOpenMotor::fraction_32_32 new_dimension_factor) {
    try {
        std::vector<uint8_t> dspeed(new_dimension_factor.nominator.array, new_dimension_factor.nominator.array +
                                                                          sizeof new_dimension_factor.nominator.array /
                                                                          sizeof new_dimension_factor.nominator.array[0]);

        std::vector<uint8_t> dtime(new_dimension_factor.denominator.array, new_dimension_factor.denominator.array +
                                                                           sizeof new_dimension_factor.denominator.array /
                                                                           sizeof new_dimension_factor.denominator.array[0]);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x604C, 0x0001, static_cast<uint32_t>(dspeed.size()),
                                 dspeed);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x604C, 0x0002, static_cast<uint32_t>(dtime.size()),
                                 dtime);

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set velocity quick stop ramp (SDO 0x604c): " << error.what() << std::endl;
    }
}

/***********************************************************************************/
uint32_t CanOpenMotor::getProfileAcceleration(uint8_t direction) {
    union_8_32 prof_acc_union(0);
    try {
        std::vector<uint8_t> prof_acc = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6083 + direction, 0x0);
        memcpy(&prof_acc_union.value, &prof_acc[0], std::min(prof_acc.size(), sizeof(uint32_t)));

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read profile/deceleration acceleration (SDO" << 0x6083 + direction << "): "
                  << error.what() << std::endl;
    }
    return prof_acc_union.value;
}

void CanOpenMotor::setProfileAcceleration(uint8_t direction, uint32_t new_accel) {
    union_8_32 accel(new_accel);
    setProfileAcceleration(direction, accel);
}

void CanOpenMotor::setProfileAcceleration(uint8_t direction, union_8_32 new_accel) {
    try {
        std::vector<uint8_t> acc(new_accel.array,
                                 new_accel.array + sizeof new_accel.array / sizeof new_accel.array[0]);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x6083 + direction, 0x0,
                                 static_cast<uint32_t>(acc.size()), acc);

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set profile/deceleration acceleration (SDO" << 0x6083 + direction << "): "
                  << error.what() << std::endl;
    }
}

/***********************************************************************************/
uint32_t CanOpenMotor::getQuickStopDeceleration() {
    union_8_32 quick_stop_deceleration(0);
    try {
        std::vector<uint8_t> prof_acc = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6085, 0x0);
        memcpy(&quick_stop_deceleration.value, &prof_acc[0], std::min(prof_acc.size(), sizeof(uint32_t)));

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read velocity acceleration/deceleration (SDO 0x6085): " << error.what() << std::endl;
    }
    return quick_stop_deceleration.value;
}

void CanOpenMotor::setQuickStopDeceleration(uint32_t new_decel) {
    union_8_32 decel(new_decel);
    setQuickStopDeceleration(decel);
}

void CanOpenMotor::setQuickStopDeceleration(CanOpenMotor::union_8_32 new_decel) {
    try {
        std::vector<uint8_t> dec(new_decel.array,
                                 new_decel.array + sizeof new_decel.array / sizeof new_decel.array[0]);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x6085, 0x0,
                                 static_cast<uint32_t>(dec.size()), dec);

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set quick stop deceleration (SDO 0x6085): " << error.what() << std::endl;
    }
}

/***********************************************************************************/
uint32_t CanOpenMotor::getMaxAcceleration(uint8_t direction) {
    union_8_32 prof_acc_union(0);
    try {
        std::vector<uint8_t> prof_acc = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x60C5 + direction, 0x0);
        memcpy(&prof_acc_union.value, &prof_acc[0], std::min(prof_acc.size(), sizeof(uint32_t)));

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read maximum acceleration/deceleration (SDO" << 0x60C5 + direction << "): "
                  << error.what() << std::endl;
    }
    return prof_acc_union.value;
}

void CanOpenMotor::setMaxAcceleration(uint8_t direction, uint32_t new_accel) {
    union_8_32 accel(new_accel);
    setMaxAcceleration(direction, accel);
}

void CanOpenMotor::setMaxAcceleration(uint8_t direction, CanOpenMotor::union_8_32 new_accel) {
    try {
        std::vector<uint8_t> acc(new_accel.array,
                                 new_accel.array + sizeof new_accel.array / sizeof new_accel.array[0]);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x60C5 + direction, 0x0,
                                 static_cast<uint32_t>(acc.size()), acc);

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set maximum acceleration/deceleration (SDO" << 0x60C5 + direction << "): "
                  << error.what() << std::endl;
    }
}

/***********************************************************************************/
int16_t CanOpenMotor::getTargetTorque() {
    try {
        return device.get_entry("target_torque");
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read target torque (SDO 0x6071/0)(torque mode): " << error.what() << std::endl;
        return -1;
    }
}

void CanOpenMotor::setTargetTorque(int16_t target_torque) {
    try {
        device.set_entry("target_torque", target_torque);
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read target torque (SDO 0x6071/0)(torque mode): " << error.what() << std::endl;
    }
}

/***********************************************************************************/
uint32_t CanOpenMotor::getMaxProfileVelocity() {
    try {
        return device.get_entry("max_profile_velocity");
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read max_profile_velocity (SDO 0x607f)(torque mode): " << error.what() << std::endl;
        return 0;
    }
}

void CanOpenMotor::setMaxProfileVelocity(uint32_t target_velocity) {
    try {
        device.set_entry("max_profile_velocity", target_velocity);
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set max_profile_velocity (SDO 0x607f)(torque mode): : " << error.what() << std::endl;
    }
}

/***********************************************************************************/
uint32_t CanOpenMotor::getMaxProfileVelocityInPPMode() {
    try {
        return device.get_entry("profile_velocity_in_pp_mode");
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read profile_velocity_in_pp_mode (SDO 0x6081)(profile velocity mode): " << error.what()
                  << std::endl;
        return 0;
    }
}

void CanOpenMotor::setMaxProfileVelocityInPPMode(uint32_t target_velocity) {
    try {
        device.set_entry("profile_velocity_in_pp_mode", target_velocity);
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set profile_velocity_in_pp_mode (SDO 0x6081)(profile velocity mode): " << error.what()
                  << std::endl;
    }
}

/***********************************************************************************/
int16_t CanOpenMotor::getQuickStopOptionCode() {
    union_8_16s qsoc_union(0);
    try {
        std::vector<uint8_t> prof_acc = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x605A, 0x0);
        memcpy(&qsoc_union.value, &prof_acc[0], std::min(prof_acc.size(), sizeof(int16_t)));

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read quick stop option code (SDO 0x605A) " << error.what() << std::endl;
    }
    return qsoc_union.value;
}

/***********************************************************************************/
int32_t CanOpenMotor::getPositionDemandValue() {
    try {
        return device.get_entry("position_demand_value");
    } catch (...) {}
    union_8_32s pdv_union(0);
    try {
        std::vector<uint8_t> prof_acc = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6062, 0x0);
        memcpy(&pdv_union.value, &prof_acc[0], std::min(prof_acc.size(), sizeof(int32_t)));

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read position demand value (SDO 0x6062) " << error.what() << std::endl;
    }
    return pdv_union.value;
}

/***********************************************************************************/
int32_t CanOpenMotor::getHomeOffset() {
    try {
        return device.get_entry("home_offset");
    } catch (...) {}
    union_8_32s ho_union(0);
    try {
        std::vector<uint8_t> prof_acc = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x607C, 0x0);
        memcpy(&ho_union.value, &prof_acc[0], std::min(prof_acc.size(), sizeof(int32_t)));

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read position demand value (SDO 0x6062) " << error.what() << std::endl;
    }
    return ho_union.value;
}

void CanOpenMotor::setHomeOffset(int32_t new_home_offset) {
    union_8_32s home_offset(new_home_offset);
    setHomeOffset(home_offset);
}

void CanOpenMotor::setHomeOffset(union_8_32s new_home_offset) {
    try {
        device.set_entry("home_offset", new_home_offset.value);
        return;
    } catch (...) {}
    try {
        std::vector<uint8_t> acc(new_home_offset.array,
                                 new_home_offset.array +
                                 sizeof new_home_offset.array / sizeof new_home_offset.array[0]);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x607C, 0x0,
                                 static_cast<uint32_t>(acc.size()), acc);

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set maximum home offset(SDO 0x607C): " << error.what() << std::endl;
    }
}

/***********************************************************************************/
uint32_t CanOpenMotor::getEndVelocity() {
    try {
        return device.get_entry("end_velocity");
    } catch (...) {}
    union_8_32 ev_union(0);
    try {
        std::vector<uint8_t> prof_acc = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6082, 0x0);
        memcpy(&ev_union.value, &prof_acc[0], std::min(prof_acc.size(), sizeof(uint32_t)));
        return ev_union.value;
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read position demand value (SDO 0x6062) " << error.what() << std::endl;
        return 0;
    }
}

void CanOpenMotor::setEndVelocity(uint32_t new_velocity) {
    union_8_32 new_vel(new_velocity);
    setEndVelocity(new_vel);
}

void CanOpenMotor::setEndVelocity(CanOpenMotor::union_8_32 new_velocity) {
    try {
        device.set_entry("end_velocity", new_velocity.value);
        return;
    } catch (...) {}
    try {
        std::vector<uint8_t> vel(new_velocity.array,
                                 new_velocity.array +
                                 sizeof new_velocity.array / sizeof new_velocity.array[0]);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x6082, 0x0,
                                 static_cast<uint32_t>(vel.size()), vel);

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set end velocity (SDO 0x6082): " << error.what() << std::endl;
    }
}

/***********************************************************************************/
int16_t CanOpenMotor::getMotionProfileType() {
    try {
        return device.get_entry("motion_profile_type");
    } catch (...) {}
    union_8_16s mp_union(0);
    try {
        std::vector<uint8_t> prof_type = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6086, 0x0);
        memcpy(&mp_union.value, &prof_type[0], std::min(prof_type.size(), sizeof(int16_t)));
        return mp_union.value;
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read motion profile type (SDO 0x6086) " << error.what() << std::endl;
        return 0;
    }
}

void CanOpenMotor::setMotionProfileType(int16_t new_motion_profile_type) {
    assert((new_motion_profile_type == 0) || (new_motion_profile_type == 1));
    union_8_16s new_vel(new_motion_profile_type);
    setMotionProfileType(new_vel);
}

void CanOpenMotor::setMotionProfileType(CanOpenMotor::union_8_16s new_motion_profile_type) {
    try {
        device.set_entry("motion_profile_type", new_motion_profile_type.value);
        return;
    } catch (...) {}
    try {
        std::vector<uint8_t> vel(new_motion_profile_type.array,
                                 new_motion_profile_type.array +
                                 sizeof new_motion_profile_type.array / sizeof new_motion_profile_type.array[0]);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x6086, 0x0,
                                 static_cast<uint32_t>(vel.size()), vel);

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set end velocity (SDO 0x6082): " << error.what() << std::endl;
    }
}

/***********************************************************************************/
int16_t CanOpenMotor::getVelocityDemandValue() {
    try {
        return device.get_entry("velocity_demand");
    } catch (...) {}
    union_8_16s vd_union(0);
    try {
        std::vector<uint8_t> vel_demand = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6043, 0x0);
        memcpy(&vd_union.value, &vel_demand[0], std::min(vel_demand.size(), sizeof(int16_t)));
        return vd_union.value;
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read velocity demand (SDO 0x6043) " << error.what() << std::endl;
        return 0;
    }
}

/***********************************************************************************/
int16_t CanOpenMotor::getVelocityActualValue() {
    try {
        return device.get_entry("velocity_actual_value");
    } catch (...) {}
    union_8_16s vav_union(0);
    try {
        std::vector<uint8_t> vel_actual_value = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6044, 0x0);
        memcpy(&vav_union.value, &vel_actual_value[0], std::min(vel_actual_value.size(), sizeof(int16_t)));
        return vav_union.value;
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read velocity actual value (SDO 0x6044) " << error.what() << std::endl;
        return 0;
    }
}

/***********************************************************************************/
uint32_t CanOpenMotor::getFollowingErrorWindow() {
    try {
        return device.get_entry("following_error_window");
    } catch (...) {}
    union_8_32 few_union(0);
    try {
        std::vector<uint8_t> few = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6065, 0x0);
        memcpy(&few_union.value, &few[0], std::min(few.size(), sizeof(uint32_t)));
        return few_union.value;
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read following error window (SDO 0x6065) " << error.what() << std::endl;
        return 0;
    }
}

void CanOpenMotor::setFollowingErrorWindow(uint32_t new_following_window) {
    union_8_32 new_window(new_following_window);
    setFollowingErrorWindow(new_window);
}

void CanOpenMotor::setFollowingErrorWindow(CanOpenMotor::union_8_32 new_following_window) {
    try {
        device.set_entry("following_error_window", new_following_window.value);
        return;
    } catch (...) {}
    try {
        std::vector<uint8_t> window(new_following_window.array,
                                    new_following_window.array +
                                    sizeof new_following_window.array / sizeof new_following_window.array[0]);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x6065, 0x0,
                                 static_cast<uint32_t>(window.size()), window);

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set following_error_window (SDO 0x6065): " << error.what() << std::endl;
    }
}

/***********************************************************************************/

uint16_t CanOpenMotor::getFollowingErrorTimeout() {
    try {
        return device.get_entry("following_error_time_out");
    } catch (...) {}
    union_8_16 feto_union(0);
    try {
        std::vector<uint8_t> feto = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6066, 0x0);
        memcpy(&feto_union.value, &feto[0], std::min(feto.size(), sizeof(uint16_t)));
        return feto_union.value;
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read following error timeout (SDO 0x6066) " << error.what() << std::endl;
        return 0;
    }
}

void CanOpenMotor::setFollowingErrorTimeout(uint16_t timeout_ms) {
    union_8_16 new_timeout(timeout_ms);
    setFollowingErrorTimeout(new_timeout);
}

void CanOpenMotor::setFollowingErrorTimeout(CanOpenMotor::union_8_16 new_following_time_out) {
    try {
        device.set_entry("following_error_time_out", new_following_time_out.value);
        return;
    } catch (...) {}
    try {
        std::vector<uint8_t> time_out(new_following_time_out.array,
                                      new_following_time_out.array +
                                      sizeof new_following_time_out.array / sizeof new_following_time_out.array[0]);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x6065, 0x0,
                                 static_cast<uint32_t>(time_out.size()), time_out);

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set following_error_timeout (SDO 0x6066): " << error.what() << std::endl;
    }
}

/***********************************************************************************/
uint32_t CanOpenMotor::getPositionWindow() {
    try {
        return device.get_entry("position_window");
    } catch (...) {}
    union_8_32 pw_union(0);
    try {
        std::vector<uint8_t> feto = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6066, 0x0);
        memcpy(&pw_union.value, &feto[0], std::min(feto.size(), sizeof(uint32_t)));
        return pw_union.value;
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read position window (SDO 0x6067) " << error.what() << std::endl;
        return 0;
    }
}

void CanOpenMotor::setPositionWindow(uint32_t new_position_window) {
    union_8_32 new_timeout(new_position_window);
    setPositionWindow(new_timeout);
}

void CanOpenMotor::setPositionWindow(CanOpenMotor::union_8_32 new_position_window) {
    try {
        device.set_entry("position_window", new_position_window.value);
        return;
    } catch (...) {}
    try {
        std::vector<uint8_t> time_out(new_position_window.array,
                                      new_position_window.array +
                                      sizeof new_position_window.array / sizeof new_position_window.array[0]);
        master.core.sdo.download(static_cast<uint8_t>(getId()), 0x6065, 0x0,
                                 static_cast<uint32_t>(time_out.size()), time_out);

    } catch (kaco::canopen_error &error) {
        std::cout << "Could not set position window (SDO 0x6067): " << error.what() << std::endl;
    }
}

/***********************************************************************************/

CanOpenMotor::~CanOpenMotor() = default;


CanOpenDevice::CanOpenDevice(size_t id_, int baudrate_, ros::NodeHandle &nh_, kaco::Master &master_) : id(id_),
                                                                                                       baudrate(
                                                                                                               baudrate_),
                                                                                                       nh(nh_),
                                                                                                       master(master_),
                                                                                                       device(master_.get_device(
                                                                                                               id_ -
                                                                                                               1)) {
    device.start();
    PRINT("Loading EDS from library...")
    const std::string loaded_eds_file = device.load_dictionary_from_library();
    PRINT("Loaded the following EDS file from the library: " << loaded_eds_file);
    PRINT("Dictionary:");
    device.read_complete_dictionary();
    device.print_dictionary();
    device.execute("enable_operation");
    op_state = OP;
}

CanOpenDevice::CanOpenDevice(size_t id_, int baudrate_, kaco::Master &master_) : id(id_),
                                                                                 baudrate(baudrate_),
                                                                                 master(master_),
                                                                                 device(master_.get_device(
                                                                                         id_ - 1)) {
    device.start();
    PRINT("Loading EDS from library...")
    const std::string loaded_eds_file = device.load_dictionary_from_library();
    PRINT("Loaded the following EDS file from the library: " << loaded_eds_file);
    PRINT("Dictionary:");
    device.read_complete_dictionary();
    device.print_dictionary();
    device.execute("enable_operation");
    op_state = OP;
}

void CanOpenDevice::set_attribute(const std::string &attribute_name, kaco::Value &value) const {
    try { device.set_entry(attribute_name, value); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not set attribute " << attribute_name << " to " << value << std::endl;
        std::cout << error.what() << std::endl;
    }
}

kaco::Value CanOpenDevice::get_attribute(const std::string &attribute_name) const {
    try { return device.get_entry(attribute_name); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not get attribute " << attribute_name << std::endl;
        std::cout << error.what() << std::endl;
    }
    return 0;
}

size_t CanOpenDevice::getId() const {
    return id;
}

void CanOpenDevice::setId(size_t id) {
    CanOpenDevice::id = id;
}

int CanOpenDevice::getBaudrate() const {
    return baudrate;
}

void CanOpenDevice::setBaudrate(int baudrate) {
    CanOpenDevice::baudrate = baudrate;
}

CanOpenDevice::~CanOpenDevice() = default;


void CanOpenDevice::EnableOperation() {
    try { device.execute("enable_operation"); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not enable operation\n";
        std::cout << error.what() << std::endl;
    }

}


void CanOpenDevice::DisableOperation() {
    try { device.execute("disable_operation"); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not disable operation\n";
        std::cout << error.what() << std::endl;
    }
}
