//
// Created by lydakis on 28/08/18.
//

#ifndef KACANOPEN_CANOPENMOTOR_H
#define KACANOPEN_CANOPENMOTOR_H

#include "master.h"
#include "canopen_error.h"
#include "logger.h"
#include "ros/ros.h"

#define DEFAULT_CALIB_RUNS 10
#define DEFAULT_STEPS_PER_ROT 1024
#define DEFAULT_MAX_VEL 5000
#define DEFAULT_MIN_VEL 60


/**
 * Wrapper class for Devices that support CANOpen
 */
class CanOpenDevice {
    /**
     * Operational states
     * PREPre-Operational
     */
    enum OpStates {
        PREOP, /**Pre-Operational State*/
        OP, /**Operational State*/
        STOPPED /**Stopped State*/
    };

public:

    /**
     * Constructor
     * @param id_ A unique positive id of the device
     * @param baudrate_ The baudrate of the CAN bus
     * @param nh_ A node handle for potential ROS integration
     * @param master kacanopen master that handles the device communication
     */
    CanOpenDevice(size_t id_, int baudrate_, ros::NodeHandle &nh_, kaco::Master &master);

    /**
     * Constructor
     * @param id_ A unique positive id of the device
     * @param baudrate_ The baudrate of the CAN bus
     * @param master kacanopen master that handles the device communication
     */
    CanOpenDevice(size_t id_, int baudrate_, kaco::Master &master);

    /**
     * Default Destructor
     */
    ~CanOpenDevice();

    /**
     * Set an attribute to a specific value
     * @param attribute_name a string containing
     * @param value a kaco::Value value for the attribute to be set to
     */
    void set_attribute(const std::string &attribute_name, kaco::Value &value) const;

    /**
     * Return the value of an attribute
     * @param attribute_name a string containing the name of the attribute
     * @return a kaco::Value value with the value of the attribute, you will need to reinterpet it on return
     */
    kaco::Value get_attribute(const std::string &attribute_name) const;

public:
    /**
     * Return the id of the node
     * @return CanOpenDevice::id
     */
    size_t getId() const;

    /**
     * Set the id of the node
     * @param id the new id of the node
     */
    void setId(size_t id);

    /**
     * Return the baudrate of the CAN bus
     * @return CanOpenDevice::baudrate
     */
    int getBaudrate() const;

    /**
     * Set the baudrate of the CAN bus
     * @param baudrate the new baudrate of the CAN bus
     */
    void setBaudrate(int baudrate);

    /**
     * Switch the device to the operational state
    */
    void EnableOperation();

    /**
    * Switch the device to the stopped state
    */
    void DisableOperation();

private:
    ///The unique id of the node
    size_t id;
    ///The baudrate of the CAN bus
    int baudrate;
    ///The operational state of the device
    OpStates op_state;

protected:
    ///A ros::NodeHandle for potential ros integration
    ros::NodeHandle nh;
    ///A kacanopen master that handles the communication with the device
    kaco::Master &master;
    ///The interface to the CanOpen Device
    kaco::Device &device;
};


/**
 * Class for 402 CANOpen compliant Nanotec devices (motors)
 */
class CanOpenMotor : public CanOpenDevice {
public:

    ///Supported motor modes
    enum MotorMode {
        UNDEFINED = -1,
        PROFILE_POSITION = 1, ///profile position mode
        VELOCITY = 2, ///velocity mode
        PROFILE_VELOCITY = 3, ///profile velocity mode
        PROFILE_TORQUE = 4 ///profile torque mode
    };

    ///Map that maps the modes to the internal string representation of kacanopen
    std::map<MotorMode, std::string> modes{
            {UNDEFINED, "undefined"},
            {PROFILE_POSITION, "profile_position_mode"},
            {VELOCITY, "velocity_mode"},
            {PROFILE_VELOCITY, "profile_velocity_mode"}
            { PROFILE_TORQUE, "profile_torque_mode" }
    };

    ///Union to book keep uint32 values as an array of bytes
    union union_8_32 {
        uint32_t value;
        uint8_t array[4];

        explicit union_8_32(uint32_t x) { value = x; }

        ~union_8_32() = default;
    };

    ///Union to book keep uint16 values as an array of bytes
    union union_8_16 {
        uint16_t value;
        uint8_t array[2];

        explicit union_8_16(uint16_t x) { value = x; }

        ~union_8_16() = default;
    };

    ///Struct to keep the acceleration or deceleration in the form of DeltaSpeed/DeltaTime
    struct velocity_accel {
        union_8_32 delta_speed; ///Change in speed
        union_8_16 delta_time; ///Change in time

        velocity_accel(uint32_t ds, uint16_t dt) : delta_speed(ds), delta_time(dt) {}

        ~velocity_accel() = default;
    };

    ///Struct to keep the minimum and maximum velocity
    struct min_max_velocity {
        union_8_32 min_velocity; ///Minimum velocity
        union_8_32 max_velocity; ///Maximum velocity

        min_max_velocity(uint32_t min, uint32_t max) : min_velocity(min), max_velocity(max) {}

        ~min_max_velocity() = default;
    };

    ///Struct to keep a fraction of 32bit numbers
    struct fraction_32_32 {
        union_8_32 nominator; ///Minimum velocity
        union_8_32 denominator; ///Maximum velocity

        fraction_32_32(uint32_t nom, uint32_t denom) : nominator(nom), denominator(denom) {}

        ~fraction_32_32() = default;
    };

    /**
     * Constructor
     * @param id_ A unique positive id of the device
     * @param baudrate_ The baudrate of the CAN bus
     * @param nh_ A node handle for potential ROS integration
     * @param master kaco::master that handles the device communication
     */
    CanOpenMotor(size_t id_, int baudrate_, ros::NodeHandle &nh_, kaco::Master &master);

    /**
     * Constructor
     * @param id_ A unique positive id of the device
     * @param baudrate_ The baudrate of the CAN bus
     * @param master kaco::master that handles the device communication
     */
    CanOpenMotor(size_t id_, int baudrate_, kaco::Master &master);

    /**
     * Default destructor
     */
    ~CanOpenMotor();

    /**
     * Return the position of the motor in steps
     * @return CanOpenMotor::current_position
     */
    int32_t getCurrent_position();

    /**
     * Set the parameter that represents the internal position to a new value (No real point)
     * @param current_position the new value for the variable that holds the internal position
     */
    void setCurrent_position(int32_t current_position);

    /**
     * Read the current position of the motor and update the
     * instance variable
     */
    void readCurrent_position();

    /**
     * Return the motor's internal position in steps
     * @return CanOpenMotor::target_position_internal
     */
    int32_t getCurrent_position_internal();

    /**
     * UNUSED
     */
    void setCurrent_position_internal(int32_t current_position_internal);

    /**
     * Read the current internal position of the motor and update CanOpenMotor::current_position_internal
     */
    void readCurrent_position_internal();

    /**
     * Return the target position in steps of the motor
     * @return the CanOpenMotor::target_position
     */
    int32_t getTarget_position();

    /**
     * Set the target position in steps of the motor in absolute steps
     * @param target_position the new target position in absolute steps
     */
    void setTarget_position(int32_t target_position);

    /**
     * Set the target position in steps of the motor in relative steps
     * @param target_position the new target position in relative steps
     */
    void setTarget_position_relative(int32_t target_position);

    /**
     * Read the current internal position of the motor and update CanOpenMotor::target_position
     */
    void readTarget_position();

    /**
     * Return the value of the target velocity stored in the instance variable
     * @return CanOpenMotor::target_velocity
     */
    int getTarget_velocity() const;

    /**
     * Set the target velocity of the motor in steps/s
     * @param target_velocity the new target velocity of the motor in steps/s
     */
    void setTarget_velocity(int target_velocity);

    /**
     * Read the target velocity from the motor and update CanOpenMotor::target_velocity
     */
    void readTarget_velocity();

    /**
     * Return the value of the current velocity stored in the instance variable
     * @return CanOpenMotor::current_velocity
     */
    int getCurrent_velocity();

    /**
     * Set the current velocity of the motor in steps/s (UNUSED)
     * @param the new value of CanOpenMotor::current_velocity
     */
    void setCurrent_velocity(int current_velocity);

    /**
     * Read the internal velocity of the motor and update CanOpenMotor::current_velocity
     */
    void readCurrent_velocity();

    /**
     * Return the status word of the motor, that describes the internal state
     * @return CanOpenMotor::status_word
     */
    int16_t getStatus_word() const;

    /**
     * Updates CanOpenMotor::status_word (UNUSED)
     * @param status_word the new value of CanOpenMotor::status_word
     */
    void setStatus_word(unsigned int status_word);

    /**
     * Read the status word from the motor and update CanOpenMotor::status_word
     */
    void readStatus_word();

    /**
     * Set the mode of operation
     * @param mode_ the new target mode for the motor
     */
    void setModeOfOperation(MotorMode mode_);

    /**
     * Read the mode of operation of the motor and update CanOpenMotor::mode
     */
    void readModeOfOperation();

    /**
     * Switch the mode to velocity mode
     * @return true if the switch was successful, false otherwise
     */
    bool enableVelocityMode();

    /**
     * Switch the mode to profile position mode
     * @return true if the switch was successful, false otherwise
     */
    bool enableProfilePositionMode();

    /**
     * Switch the mode to profile torque mode
     * @return true if the switch was successful, false otherwise
     */
    bool enableProfileTorqueMode();

    /**
     * Return the mode of operation
     * @return CanOpenMotor::mode
     */
    CanOpenMotor::MotorMode getModeOfOperation();

    /**
     * Return the output of the ramp generator, which simultaneously serves as the preset value for the speed controller.
     * Use in Velocity / Profile Velocity Mode
     * @return the output of the ramp generator, which simultaneously serves as the preset value for the speed controller.
     */
    int32_t getVelocityDemandValue();

    /**
     * Set the value of the velocity demand value
     * @param vel_demand_val the new value for velocity demand
     */
    void setVelocityDemandValue(int32_t vel_demand_val);


    /**
     * Return the target velocity threshold for Profile Velocity Mode
     * @return the target velocity threshold
     */
    uint16_t getVelocityWindow();

    /**
     * Set the target velocity threshold for Profile Velocity Mode
     * @param velocity_window the new velocity threshold
     */
    void setVelocityWindow(int16_t velocity_window);

    /**
     * Return the velocity acceleration or deceleration
     * @param direction 0 for acceleration 1 for deceleration
     * @return a CanOpenMotor::velocity_accel struct with the motor settings
     */
    velocity_accel getVelocityAcceleration(uint8_t direction);

    /**
     * Set the velocity acceleration or deceleration
     * @param direction 0 for acceleration 1 for deceleration
     * @param new_val a CanOpenMotor::velocity_accel struct with the read parameters
     */
    void setVelocityAcceleration(uint8_t direction, velocity_accel new_val);

    /**
     * Return the min/max velocity limits of the motor
     * @return a CanOpenMotor::min_max_velocity struct with the motor settings
     */
    min_max_velocity getMinMaxVelocity();

    /**
     * Set the min/max velocity limits of the motor
     * @param min the minimum allowed target velocity in steps/s
     * @param max the maximum allowed target velocity in steps/s
     */
    void setMinMaxVelocity(uint32_t min, uint32_t max);

    /**
     * Set the deceleration (deceleration ramp) if the Quick Stop state is initiated in Velocity Mode.
     * @param delta_speed change in units
     * @param delta_time change in time
     */
    void setQuickStopRamp(uint32_t delta_speed, uint16_t delta_time);

    /**
     * Set the deceleration (deceleration ramp) if the Quick Stop state is initiated in Velocity Mode.
     * @param new_ramp a CanOpenMotor::velocity_accel struct containing the new values
     */
    void setQuickStopRamp(velocity_accel new_ramp);

    /**
     * Read the deceleration (deceleration ramp) if the Quick Stop state is initiated in Velocity Mode as <speed change> / <time change>
     * @return a CanOpenMotor::velocity_accel struct with the read parameters
     */
    velocity_accel getQuickStopRamp();

    /**
     * @brief Read the dimension factor for velocity mode
     * If subindex 1 is set to the value "1" and subindex 2 is set to the value "1";
     * the speed is specified in revolutions per minute.
     * Otherwise, subindex 1 contains the denominator (multiplier) and subindex 2 contains the numerator (divisor)
     * with which the internal speed values are converted to revolutions per second.
     * If subindex 1 is set to the value "1" and subindex 2 is set to the value "60" (factory setting),
     * the speed is specified in revolutions per minute (1 revolution per 60 seconds).
     * @return a CanOpenMotor::velocity_accel struct with the read parameters
     */
    fraction_32_32 getDimensionFactor();

    /**
     * Set the the dimension factor for velocity mode
     * @param nominator the fraction nominator
     * @param denominator the fraction denominator
     */
    void setDimensionFactor(uint32_t nominator, uint32_t denominator);

    /**
     * Set the the dimension factor for velocity mode
     * @param new_ramp a CanOpenMotor::fraction_32_32 struct containing the new values
     */
    void setDimensionFactor(fraction_32_32 new_dimension_factor);

    /**
     * Get the maximum desired starting acceleration or braking decelerationin user-defined units
     * @param direction 0 for acceleration, 1 for deceleration
     * @return the the maximum acceleration in user-defined units
     */
    uint32_t getProfileAcceleration(uint8_t direction);

    /**
     * Set the maximum desired starting acceleration or braking decelerationin user-defined units
     * @param direction 0 for acceleration, 1 for deceleration
     * @param new_accel the new maximum acceleration
     */
    void setProfileAcceleration(uint8_t direction, uint32_t new_accel);

    /**
     * Set the maximum desired starting acceleration or braking decelerationin user-defined units
     * @param direction 0 for acceleration, 1 for deceleration
     * @param new_accel a CanOpenMotor::union_8_32 containing the new maximum acceleration
     */
    void setProfileAcceleration(uint8_t direction, union_8_32 new_accel);

    /**
     * Get the maximum Quick Stop Deceleration in user-defined units.
     * @return the maximum Quick Stop Deceleration in user-defined units.
     */
    uint32_t getQuickStopDeceleration();

    /**
     * Set the maximum Quick Stop Deceleration in user-defined units.
     * @param new_decel the new maximum acceleration
     */
    void setQuickStopDeceleration(uint32_t new_decel);

    /**
     * Set the maximum Quick Stop Deceleration in user-defined units.
     * @param new_decel a CanOpenMotor::union_8_32 containing the new maximum deceleration
     */
    void setQuickStopDeceleration(union_8_32 new_decel);

    /**
    * Return the maximum acceleration that may not be exceeded when moving to the end position or
    * the maximum braking deceleration that may not be exceeded when moving to the end position
    * @param direction 0 for acceleration, 1 for deceleration
    * @return the the maximum acceleration in user-defined units
    */
    uint32_t getMaxAcceleration(uint8_t direction);

    /**
     * Set the maximum acceleration that may not be exceeded when moving to the end position or
     * the maximum braking deceleration that may not be exceeded when moving to the end position
     * @param direction 0 for acceleration, 1 for deceleration
     * @param new_accel the new maximum acceleration
     */
    void setMaxAcceleration(uint8_t direction, uint32_t new_accel);

    /**
     * Set the maximum acceleration that may not be exceeded when moving to the end position or
     * the maximum braking deceleration that may not be exceeded when moving to the end position
     * @param direction 0 for acceleration, 1 for deceleration
     * @param new_accel a CanOpenMotor::union_8_32 containing the new maximum acceleration
     */
    void setMaxAcceleration(uint8_t direction, union_8_32 new_accel);

    /**
     * @brief Return the target torque (Torque Mode)
     * Return the target value for torque to be set. The torque is directly proportionate to the
     * current, which is why the value is specified in thousands of maximum settable current.
     * @return the target torque
     */
    int16_t getTargetTorque();

    /**
     * @brief set the target torque (Torque Mode)
     * Set the target value for the torque to be set. The torque is directly proportionate to the
     * current, which is why the value is specified in thousands of maximum settable current.
     * @param target_torque the new target torque
     */
    void setTargetTorque(int16_t target_torque);

    /**
     * @brief return the max profile velocity (Torque Mode) in RPM
     * Return the maximum permissible rotational speed as an amount for both rotational
     * directions, which can be set in Torque mode in RPM
     * @return return the max profile velocity (Torque Mode) in RPM
     */
    uint32_t getMaxProfileVelocity();

    /**
     * Run 10 full rotations of the motor to calculate the
     * steps needed for one full rotation
     */
    void calibrate();

    /**
     * Print the current parameters of the motor
     */
    void print();

protected:

private:
    ///The current position of the motor in steps, since the motor was turned on
    int32_t current_position = 0;
    ///The current position of the motor in total steps
    int32_t current_position_internal = 0;
    ///The target position in steps, could be absolute or relative to current position, for profile position mode
    int32_t target_position = 0;
    ///The target velocity in steps/s, for velocity mode
    int16_t target_velocity = 0;
    ///The current velocity of the motor in steps/
    int16_t current_velocity = 0;
    ///The status word of the motor
    int16_t status_word = 0;
    ///The operating mode of the motor
    CanOpenMotor::MotorMode mode = VELOCITY;
    ///Steps per full rotation
    size_t steps_per_rotation = 0;

};

#endif //KACANOPEN_CANOPENMOTOR_H
