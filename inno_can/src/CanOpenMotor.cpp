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

int32_t CanOpenMotor::getCurrent_position() {
    readCurrent_position();
    return current_position;
}

void CanOpenMotor::setCurrent_position(int32_t current_position) {
    CanOpenMotor::current_position = current_position;
}

int32_t CanOpenMotor::getCurrent_position_internal() {
    try { readCurrent_position_internal(); } catch (...) {}
    return current_position_internal;
}

void CanOpenMotor::setCurrent_position_internal(int32_t current_position_internal) {
    CanOpenMotor::current_position_internal = current_position_internal;
}

int32_t CanOpenMotor::getTarget_position() {
    readTarget_position();
    return target_position;
}

void CanOpenMotor::setTarget_position(int32_t target_position) {
//    device.set_entry("target_position", target_position);
    device.execute("set_target_position", target_position);
    readTarget_position();
}

int CanOpenMotor::getTarget_velocity() const {
    return target_velocity;
}

void CanOpenMotor::setTarget_velocity(int target_velocity) {
    device.execute("set_target_velocity", (int16_t) target_velocity);
    readTarget_velocity();
}

int CanOpenMotor::getCurrent_velocity() {
    readTarget_velocity();
    return current_velocity;
}

void CanOpenMotor::setCurrent_velocity(int current_velocity) {
    CanOpenMotor::current_velocity = static_cast<int16_t>(current_velocity);
}

int16_t CanOpenMotor::getStatus_word() const {
    return status_word;
}

void CanOpenMotor::setStatus_word(unsigned int status_word) {
    CanOpenMotor::status_word = static_cast<int16_t>(status_word);
}

void CanOpenMotor::readCurrent_position() {
    try { current_position = int32_t(get_attribute("position_actual_value")); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not read position\n";
        std::cout << error.what() << std::endl;
    }
}

void CanOpenMotor::readCurrent_position_internal() {
    try { current_position_internal = int32_t(get_attribute("position_actual_value*")); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not read internal position\n";
        std::cout << error.what() << std::endl;
    }
}

void CanOpenMotor::readTarget_position() {
    try { target_position = int32_t(get_attribute("target_position")); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not read target position\n";
        std::cout << error.what() << std::endl;
    }

}

void CanOpenMotor::readTarget_velocity() {
    try { target_velocity = int16_t(get_attribute("vl_target_velocity")); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not read target velocity\n";
        std::cout << error.what() << std::endl;
    }
}

void CanOpenMotor::readCurrent_velocity() {
    try { current_velocity = int16_t(get_attribute("velocity_actual_value")); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not read current velocity\n";
        std::cout << error.what() << std::endl;
    }
}

void CanOpenMotor::readStatus_word() {
    try { status_word = int16_t(get_attribute("status_word")); }
    catch (const kaco::canopen_error &error) {
        std::cout << "Could not read status word\n";
        std::cout << error.what() << std::endl;
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

void CanOpenMotor::setTarget_position_relative(int32_t target_position_) {
    device.execute("set_target_position_relative", target_position_);
    readTarget_position();
}

void CanOpenMotor::calibrate() {
    std::cout << "Calibrating motor\n";
    int sum = 0;
    setModeOfOperation(PROFILE_POSITION);
    assert(getModeOfOperation() == PROFILE_POSITION);
    readCurrent_position_internal();
    readCurrent_position();
    try {
        for (size_t i = 0; i < DEFAULT_CALIB_RUNS; i++) {
            long starting_position = getCurrent_position();
            this->setTarget_position_relative(int32_t(1024));
            usleep(2000000);
            long diff = getCurrent_position() - starting_position;
            std::cout << "Calibration run " << i + 1 << " starting in position " << starting_position
                      << " and ending in position " << getCurrent_position() << std::endl;
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

int32_t CanOpenMotor::getVelocityDemandValue() {
    return device.get_entry("vl_velocity_demand");
}

uint16_t CanOpenMotor::getVelocityWindow() {
    return device.get_entry("velocity_window");
}

void CanOpenMotor::setVelocityDemandValue(int32_t vel_demand_val) {
    device.set_entry("vl_velocity_demand", vel_demand_val);
}

void CanOpenMotor::setVelocityWindow(int16_t velocity_window) {
    device.set_entry("vl_velocity_window", velocity_window);
}

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
        std::cout << "Error while reading velocity acceleration/deceleration: " << error.what();
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
        std::cout << "Error while setting velocity acceleration/deceleration: " << error.what();
    }
}

CanOpenMotor::min_max_velocity CanOpenMotor::getMinMaxVelocity() {
    min_max_velocity mmv{0, 0};
    try {
        std::vector<uint8_t> min_vel = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6046, 0x0001);
        memcpy(&mmv.min_velocity.value, &min_vel[0], std::min(min_vel.size(), sizeof(uint32_t)));

        std::vector<uint8_t> max_vel = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6046, 0x0002);
        memcpy(&mmv.max_velocity.value, &max_vel[0], std::min(max_vel.size(), sizeof(uint16_t)));
    } catch (kaco::canopen_error &error) {
        std::cout << "Error while reading min/max velocity limits: " << error.what();
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
        std::cout << "Error while setting min/max velocity limits: " << error.what();
    }
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
        std::cout << "Error while setting velocity quick stop ramp: " << error.what();
    }
}

CanOpenMotor::velocity_accel CanOpenMotor::getQuickStopRamp() {
    velocity_accel qs = {0, 0};
    try {
        std::vector<uint8_t> delta_speed = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x604A, 0x0001);
        memcpy(&qs.delta_speed.value, &delta_speed[0], std::min(delta_speed.size(), sizeof(uint32_t)));

        std::vector<uint8_t> delta_time = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x604A, 0x0002);
        memcpy(&qs.delta_time.value, &delta_time[0], std::min(delta_time.size(), sizeof(uint16_t)));
    } catch (kaco::canopen_error &error) {
        std::cout << "Error while reading velocity acceleration/deceleration: " << error.what();
    }
    return qs;
}

CanOpenMotor::fraction_32_32 CanOpenMotor::getDimensionFactor() {
    fraction_32_32 qs = {0, 0};
    try {
        std::vector<uint8_t> delta_speed = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x604C, 0x0001);
        memcpy(&qs.nominator.value, &delta_speed[0], std::min(delta_speed.size(), sizeof(uint32_t)));

        std::vector<uint8_t> delta_time = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x604C, 0x0002);
        memcpy(&qs.denominator.value, &delta_time[0], std::min(delta_time.size(), sizeof(uint16_t)));
    } catch (kaco::canopen_error &error) {
        std::cout << "Error while reading velocity acceleration/deceleration: " << error.what();
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
        std::cout << "Error while setting velocity quick stop ramp: " << error.what();
    }
}

uint32_t CanOpenMotor::getProfileAcceleration(uint8_t direction) {
    union_8_32 prof_acc_union(0);
    try {
        std::vector<uint8_t> prof_acc = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6083 + direction, 0x0);
        memcpy(&prof_acc_union.value, &prof_acc[0], std::min(prof_acc.size(), sizeof(uint32_t)));

    } catch (kaco::canopen_error &error) {
        std::cout << "Error while reading velocity acceleration/deceleration: " << error.what();
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
        std::cout << "Error while setting profile acceleration: " << error.what();
    }
}

uint32_t CanOpenMotor::getQuickStopDeceleration() {
    union_8_32 quick_stop_deceleration(0);
    try {
        std::vector<uint8_t> prof_acc = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x6085, 0x0);
        memcpy(&quick_stop_deceleration.value, &prof_acc[0], std::min(prof_acc.size(), sizeof(uint32_t)));

    } catch (kaco::canopen_error &error) {
        std::cout << "Error while reading velocity acceleration/deceleration: " << error.what();
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
        std::cout << "Error while setting quick stop deceleration: " << error.what();
    }
}

uint32_t CanOpenMotor::getMaxAcceleration(uint8_t direction) {
    union_8_32 prof_acc_union(0);
    try {
        std::vector<uint8_t> prof_acc = master.core.sdo.upload(static_cast<uint8_t>(getId()), 0x60C5 + direction, 0x0);
        memcpy(&prof_acc_union.value, &prof_acc[0], std::min(prof_acc.size(), sizeof(uint32_t)));

    } catch (kaco::canopen_error &error) {
        std::cout << "Error while reading velocity acceleration/deceleration: " << error.what();
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
        std::cout << "Error while setting maximum acceleration/deceleration: " << error.what();
    }
}

int16_t CanOpenMotor::getTargetTorque() {
    try {
        return device.get_entry("target_torque");
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read target torque: " << error.what() << std::endl;
        return -1;
    }
}

void CanOpenMotor::setTargetTorque(int16_t target_torque) {
    try {
        device.set_entry("target_torque", target_torque);
    } catch (kaco::canopen_error &error) {
        std::cout << "Could not read target torque: " << error.what() << std::endl;
    }
}

uint32_t CanOpenMotor::getMaxProfileVelocity() {
    try {
        return device.get_entry("max_profile_velocity");
    } catch (kaco::canopen_error &error) {
        std::cout << "max_profile_velocity: " << error.what() << std::endl;
        return 0;
    }
}

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
