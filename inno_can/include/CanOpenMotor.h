//
// Created by lydakis on 28/08/18.
//

#ifndef KACANOPEN_CANOPENMOTOR_H
#define KACANOPEN_CANOPENMOTOR_H

#include "master.h"
#include "canopen_error.h"
#include "logger.h"
#include "ros/ros.h"
#include "device_rpdo.h"
#include "device_tpdo.h"
#include "parse_sdo.h"

#define DEFAULT_CALIB_RUNS 10
#define DEFAULT_STEPS_PER_ROT 1024
#define DEFAULT_MAX_VEL 5000
#define DEFAULT_MIN_VEL 60


/**
 * Wrapper class for Devices that support CANOpen
 */
class CanOpenDevice {
    /**
     * @brief Operational states
     */
    enum OpStates {
        PREOP, /**Pre-Operational State*/
        OP, /**Operational State*/
        STOPPED /**Stopped State*/
    };

public:

    /**
     * @brief Constructor
     * @param id_ A unique positive id of the device
     * @param baudrate_ The baudrate of the CAN bus
     * @param nh_ A node handle for potential ROS integration
     * @param master kacanopen master that handles the device communication
     */
    CanOpenDevice(size_t id_, int baudrate_, ros::NodeHandle &nh_, kaco::Master &master);

    /**
     * @brief Constructor
     * @param id_ A unique positive id of the device
     * @param baudrate_ The baudrate of the CAN bus
     * @param master kacanopen master that handles the device communication
     */
    CanOpenDevice(size_t id_, int baudrate_, kaco::Master &master);

    /**
     * @brief Default Destructor
     */
    ~CanOpenDevice();

    /**
     * @brief Set an attribute to a specific value
     * @param attribute_name a string containing
     * @param value a kaco::Value value for the attribute to be set to
     */
    void set_attribute(const std::string &attribute_name, kaco::Value &value) const;

    /**
     * @brief Return the value of an attribute
     * @param attribute_name a string containing the name of the attribute
     * @return a kaco::Value value with the value of the attribute, you will need to reinterpet it on return
     */
    kaco::Value get_attribute(const std::string &attribute_name) const;

public:
    /**
     * @brief Return the id of the node
     * @return CanOpenDevice::id
     */
    size_t getId() const;

    /**
     * @brief Set the id of the node
     * @param id the new id of the node
     */
    void setId(size_t id);

    /**
     * @brief Return the baudrate of the CAN bus
     * @return CanOpenDevice::baudrate
     */
    int getBaudrate() const;

    /**
     * @brief Set the baudrate of the CAN bus
     * @param baudrate the new baudrate of the CAN bus
     */
    void setBaudrate(int baudrate);

    /**
     * @brief Switch the device to the operational state
    */
    void EnableOperation();

    /**
    * @brief Switch the device to the stopped state
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
        ///undefined error value
                UNDEFINED = -1,
        ///profile position mode
                PROFILE_POSITION = 1,
        ///velocity mode
                VELOCITY = 2,
        ///profile velocity mode
                PROFILE_VELOCITY = 3,
        ///profile torque mode
                PROFILE_TORQUE = 4
    };

    ///Map that maps the modes to the internal string representation of kacanopen
    std::map<MotorMode, std::string> modes{
            {UNDEFINED,        "undefined"},
            {PROFILE_POSITION, "profile_position_mode"},
            {VELOCITY,         "velocity_mode"},
            {PROFILE_VELOCITY, "profile_velocity_mode"},
            {PROFILE_TORQUE,   "profile_torque_mode"}
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

    ///Union to book keep int32 values as an array of bytes
    union union_8_32s {
        int32_t value;
        uint8_t array[4];

        explicit union_8_32s(int32_t x) { value = x; }

        ~union_8_32s() = default;
    };

    ///Union to book keep int16 values as an array of bytes
    union union_8_16s {
        int16_t value;
        uint8_t array[2];

        explicit union_8_16s(int16_t x) { value = x; }

        ~union_8_16s() = default;
    };

    ///Struct to keep the acceleration or deceleration in the form of DeltaSpeed/DeltaTime
    struct velocity_accel {
        ///Change in speed
        union_8_32 delta_speed;
        ///Change in time
        union_8_16 delta_time;

        velocity_accel(uint32_t ds, uint16_t dt) : delta_speed(ds), delta_time(dt) {}

        ~velocity_accel() = default;
    };

    ///Struct to keep the minimum and maximum velocity
    struct min_max_velocity {
        ///Minimum velocity
        union_8_32 min_velocity;
        ///Maximum velocity
        union_8_32 max_velocity;

        ///Constructor
        min_max_velocity(uint32_t min, uint32_t max) : min_velocity(min), max_velocity(max) {}

        ///Destructor
        ~min_max_velocity() = default;
    };

    ///Struct to keep the signed minimum and maximum velocity
    struct signed_min_max_velocity {
        ///Minimum velocity
        union_8_32s min_velocity;
        ///Maximum velocity
        union_8_32s max_velocity;

        ///Constructor
        signed_min_max_velocity(int32_t min, int32_t max) : min_velocity(min), max_velocity(max) {}

        ///Destructor
        ~signed_min_max_velocity() = default;
    };

    ///Struct to keep a fraction of 32bit numbers
    struct fraction_32_32 {
        ///Fraction Nominator
        union_8_32 nominator;
        ///Fraction Denominator
        union_8_32 denominator;

        fraction_32_32(uint32_t nom, uint32_t denom) : nominator(nom), denominator(denom) {}

        ~fraction_32_32() = default;
    };

    ///Struct to keep the encoder settings
    struct encoder_struct {
        ///encoder increments
        union_8_32 encoder_increments;
        ///motor revolutions
        union_8_32 motor_revolutions;

        ///COnstructor
        encoder_struct(uint32_t ei, uint32_t mr) : encoder_increments(ei), motor_revolutions(mr) {}

        ///Destructor
        ~encoder_struct() = default;
    };

    /**
     * @brief Constructor
     * @param id_ A unique positive id of the device
     * @param baudrate_ The baudrate of the CAN bus
     * @param nh_ A node handle for potential ROS integration
     * @param master kaco::master that handles the device communication
     */
    CanOpenMotor(size_t id_, int baudrate_, ros::NodeHandle &nh_, kaco::Master &master);

    /**
     * @brief Constructor
     * @param id_ A unique positive id of the device
     * @param baudrate_ The baudrate of the CAN bus
     * @param master kaco::master that handles the device communication
     */
    CanOpenMotor(size_t id_, int baudrate_, kaco::Master &master);

    /**
     * @brief Default destructor
     */
    ~CanOpenMotor();


    /********** PROFILE VELOCITY MODE (not supported for sdci36 *************/

    /**
     * @brief Return a symmetrical range relative to the target speed within which the target is considered having been met
     * @return the target velocity threshold
     */
    uint16_t getVelocityWindow();

    /**
     * @brief Set a symmetrical range relative to the target speed within which the target is considered having been met
     */
    void setVelocityWindow(int16_t velocity_window);

    /**************************** VELOCITY MODE *****************************/

    /**
     * @brief Return the target speed in steps/s (SDO 0x6042).
     *
     * Values greater than 0 stand for right-hand rotation, values less than 0 stand for left-
     * hand rotation.
     *
     * The direction can be reversed with the “Polarity” object.
     *
     * @return CanOpenMotor::target_velocity
     */
    int getTargetVelocity() const;

    /**
    * @brief Set the target speed in steps/s (SDO 0x6042).
    *
    * Values greater than 0 stand for right-hand rotation, values less than 0 stand for left-
    * hand rotation.
    *
    * The direction can be reversed with the “Polarity” object.
    * @param target_velocity the new target velocity of the motor in steps/s
    */
    void setTargetVelocity(int target_velocity);

    /**
     * @brief Read the target velocity from the motor and update CanOpenMotor::target_velocity
     */
    void readTargetVelocity();

    /**
     * @brief Return the value of the current velocity stored in the instance variable
     * @return CanOpenMotor::current_velocity
     */
    int getCurrentVelocity();

    /**
     * @brief Set the current velocity of the motor in steps/s (UNUSED)
     * @param the new value of CanOpenMotor::current_velocity
     */
    void setCurrentVelocity(int current_velocity);

    /**
     * @brief Read the internal velocity of the motor and update CanOpenMotor::current_velocity
     */
    void readCurrentVelocity();

    /**
     * @brief Return the current target speed in steps/s (SDO 0x6043).
     * @return the current target speed in steps/s
     */
    int16_t getVelocityDemandValue();

    /**
     * @brief Return the current actual speed in steps/s (SDO 0x6044).
     *
     * A value is only output when the closed loop is activated.
     * @return the current actual speed in steps/s.
     */
    int16_t getVelocityActualValue();

    /**
     * @brief Return the min/max velocity limits of the motor (0x6046)
     *
     * The minimum speed and maximum speed in steps/s can be set with this object.
     *
     * Subindex 1 contains the minimum speed.\n
     * Subindex 2 contains the maximum speed.
     *
     * If the magnitude of a target speed (SDO 0x6042) is less than the minimum speed, the
     * minimum speed applies. If the target speed is 0, the motor stops.
     *
     * A target speed greater than the maximum speed sets the speed to the maximum
     * speed and sets bit 11 (internal limit active) in the status word (SDO 0x6041).
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
    * @brief Return the velocity acceleration or deceleration (SDO 0x6048)
     *
    * Sets the acceleration ramp in VL mode.
    * The acceleration is specified as a fraction:
    *
    * Speed change per time change.\n
    * Subindex 1 contains the speed change in steps/s (u32).\n
    * Subindex 2 contains the time change in s (u16).
    *
    * Neither the numerator nor the denominator must be set to 0.
    * @param direction 0 for acceleration 1 for deceleration
    * @return a CanOpenMotor::velocity_accel struct with the motor settings
    */
    velocity_accel getVelocityAcceleration(uint8_t direction);

    /**
     * @brief Set the velocity acceleration or deceleration
     * @param direction 0 for acceleration 1 for deceleration
     * @param new_val a CanOpenMotor::velocity_accel struct with the read parameters
     */
    void setVelocityAcceleration(uint8_t direction, velocity_accel new_val);

    /**
     * @brief Read the deceleration (deceleration ramp) if the Quick Stop state is initiated in Velocity Mode as <speed change> / <time change> (SDO 0x604A).s
     * Sets the braking ramp for the quick stop in VL mode.
     * The acceleration is specified as a fraction:
     *
     * Speed change per time change.\n
     * Subindex 1 contains the speed change in steps/s (u32).\n
     * Subindex 2 contains the time change in s (u16)
     *
     * Neither the numerator nor the denominator must be set to 0.
     * @return a CanOpenMotor::velocity_accel struct with the read parameters
     */
    velocity_accel getQuickStopRamp();

    /**
    * @brief Set the deceleration (deceleration ramp) if the Quick Stop state is initiated in Velocity Mode.
    * @param delta_speed change in units
    * @param delta_time change in time
    */
    void setQuickStopRamp(uint32_t delta_speed, uint16_t delta_time);

    /**
     * @brief Set the deceleration (deceleration ramp) if the Quick Stop state is initiated in Velocity Mode.
     * @param new_ramp a CanOpenMotor::velocity_accel struct containing the new values
     */
    void setQuickStopRamp(velocity_accel new_ramp);

    /**
     * @brief Read the dimension factor for velocity mode (SDO 0x604C).
     *
     * If subindex 1 is set to the value "1" and subindex 2 is set to the value "1";
     * the speed is specified in revolutions per minute.
     *
     * Otherwise, subindex 1 contains the denominator (multiplier) and subindex 2 contains the numerator (divisor)
     * with which the internal speed values are converted to revolutions per second.
     *
     * If subindex 1 is set to the value "1" and subindex 2 is set to the value "60" (factory setting),
     * the speed is specified in revolutions per minute (1 revolution per 60 seconds).
     * @return a CanOpenMotor::velocity_accel struct with the read parameters
     */
    fraction_32_32 getDimensionFactor();

    /**
     * @brief Set the the dimension factor for velocity mode (SDO 0x604C).
     * @param nominator the fraction nominator
     * @param denominator the fraction denominator
     */
    void setDimensionFactor(uint32_t nominator, uint32_t denominator);

    /**
     * @brief Set the the dimension factor for velocity mode (SDO 0x604C).
     * @param new_ramp a CanOpenMotor::fraction_32_32 struct containing the new values
     */
    void setDimensionFactor(fraction_32_32 new_dimension_factor);

    /************************ PROFILE POSITION MODE *************************/

    /**
     * @brief Return the position of the motor in steps
     *
     * Contains the current encoder position (SDO 0x6064).
     * @return CanOpenMotor::current_position
     */
    int32_t getCurrentPosition();

    /**
     * @brief Set the parameter that represents the internal position to a new value (No real point)
     * @param current_position the new value for the variable that holds the internal position
     */
    void setCurrentPosition(int32_t current_position);

    /**
     * @brief Read the current position of the motor and update CanOpenMotor::current_position (SDO 0x6064)
     */
    void readCurrentPosition();

    /**
     * @brief Return the motor's internal position in steps (SDO 0x6063)
     *
     * Contains the current actual position (encoder position converted acc. to Feed Constant and Gear Ratio).
     * @return CanOpenMotor::target_position_internal
     */
    int32_t getCurrentPositionInternal();

    /**
     * @brief UNUSED
     */
    void setCurrentPositionInternal(int32_t current_position_internal);

    /**
     * @brief Read the current internal position of the motor and update CanOpenMotor::current_position_internal
     */
    void readCurrent_position_internal();


    /**
     * @brief Return the target position in steps of the motor (SDO 0x607A)
     *
     * Specifies the target position.
     *
     * Depending on the command of the control word, the end position is interpreted as
     * relative to the current position or absolute to the reference position.
     *
     * The direction can be reversed with the object 0x607E (polarity)
     * @return the CanOpenMotor::target_position
     */
    int32_t getTargetPosition();

    /**
     * @brief Set the target position in steps of the motor in absolute steps
     * @param target_position the new target position in absolute steps
     */
    void setTargetPosition(int32_t target_position);

    /**
     * @brief Set the target position in steps of the motor in relative steps
     * @param target_position the new target position in relative steps
     */
    void setTargetPositionRelative(int32_t target_position);

    /**
     * @brief Read the current internal position of the motor and update CanOpenMotor::target_position
     */
    void readTargetPosition();

    /**
     * @brief Specifies the maximum traveling speed in steps per second (Profile Position Mode).
     * @return the maximum traveling speed in steps per second
     */
    uint32_t getMaxProfileVelocityInPPMode();

    /**
     * @brief Set the max profile velocity (Profile Position) in steps per second
     * @param target_velocity
     */
    void setMaxProfileVelocityInPPMode(uint32_t target_velocity);

    /**
     * @brief Get the maximum desired starting acceleration or braking deceleration in user-defined units (Profile Position Mode)
     * @param direction 0 for acceleration, 1 for deceleration
     * @return the the maximum acceleration in user-defined units
     */
    uint32_t getProfileAcceleration(uint8_t direction);

    /**
     * @brief Set the maximum desired starting acceleration or braking deceleration in user-defined units (Profile Position Mode)
     * @param direction 0 for acceleration, 1 for deceleration
     * @param new_accel the new maximum acceleration
     */
    void setProfileAcceleration(uint8_t direction, uint32_t new_accel);

    /**
     * @brief Set the maximum desired starting acceleration or braking deceleration in user-defined units (Profile Position Mode)
     * @param direction 0 for acceleration, 1 for deceleration
     * @param new_accel a CanOpenMotor::union_8_32 containing the new maximum acceleration
     */
    void setProfileAcceleration(uint8_t direction, union_8_32 new_accel);

    /**
     * @brief Get the maximum Quick Stop Deceleration in user-defined units.
     * @return the maximum Quick Stop Deceleration in user-defined units.
     */
    uint32_t getQuickStopDeceleration();

    /**
     * @brief Set the maximum Quick Stop Deceleration in user-defined units.
     * @param new_decel the new maximum acceleration
     */
    void setQuickStopDeceleration(uint32_t new_decel);

    /**
     * @brief Set the maximum Quick Stop Deceleration in user-defined units.
     * @param new_decel a CanOpenMotor::union_8_32 containing the new maximum deceleration
     */
    void setQuickStopDeceleration(union_8_32 new_decel);

    /**
     * @brief Return the behavior upon a Quick Stop (SDO 0x605A).
     *
     * Braking is currently only supported with
     * maximum current and subsequent change to “Switch On Disabled”.
     * @return the quick stop option code
     */
    int16_t getQuickStopOptionCode();

    /**
     * @brief Return the current demanded position (SDO 0x6062).
     * @return the current demanded position
     */
    int32_t getPositionDemandValue();


    /**
     * @brief Return the difference between the zero position of the application and the reference point of the machine (SDO 0x607C).
     * @return the difference between the zero position of the application and the reference point of the machine
     */
    int32_t getHomeOffset();

    /**
     * @brief Set the difference between the zero position of the application and the reference point of the machine (SDO 0x607C).
     * @param new_home_offset the new difference
     */
    void setHomeOffset(int32_t new_home_offset);

    /**
     * @brief Set the difference between the zero position of the application and the reference point of the machine (SDO 0x607C).
     * @param new_home_offset the new difference
     */
    void setHomeOffset(union_8_32s new_home_offset);

    /**
     * @brief Return the minimum traveling speed for a trapezoidal ramp in steps per second (SDO 0x6082).
     * @return the minimum traveling speed for a trapezoidal ramp in steps per second
     */
    uint32_t getEndVelocity();

    /**
     * @brief Set the minimum traveling speed for a trapezoidal ramp in steps per second (SDO 0x6082).
     * @param new_velocity the new end velocity
     */
    void setEndVelocity(uint32_t new_velocity);

    /**
     * @brief Set the minimum traveling speed for a trapezoidal ramp in steps per second (SDO 0x6082).
     * @param new_velocity the new end velocity as a CanOpenMotor::union_8_32 struct
     */
    void setEndVelocity(union_8_32 new_velocity);

    /***************************** TORQUE MODE ******************************/

    /**
     * @brief Return the target torque (Torque Mode)
     *
     * Return the target value for torque to be set. The torque is directly proportionate to the
     * current,\n which is why the value is specified in thousands of maximum settable current.
     * @return the target torque
     */
    int16_t getTargetTorque();

    /**
     * @brief set the target torque (Torque Mode)
     *
     * Set the target value for the torque to be set. The torque is directly proportionate to the
     * current,\n which is why the value is specified in thousands of maximum settable current.
     * @param target_torque the new target torque
     */
    void setTargetTorque(int16_t target_torque);

    /**
     * @brief return the max profile velocity (Torque Mode) in RPM
     *
     * Return the maximum permissible rotational speed as an amount for both rotational
     * directions,\n which can be set in Torque mode in RPM
     * @return return the max profile velocity (Torque Mode) in RPM
     */
    uint32_t getMaxProfileVelocity();

    /**
     * @brief Set the max profile velocity (Torque Mode) in RPM
     * @param target_velocity the new target velocity in RPM
     */
    void setMaxProfileVelocity(uint32_t target_velocity);

    /******************** NON MODE SPECIFIC SDOs ***************************/

    /**
     * @brief Return the ramp type.
     *
     * Currently only a Sin2 (value=1) and a linear/trapezoidal ramp is supported (value = 0)
     * (SDO 0x6086)
     * @return 1 or 0
     */
    int16_t getMotionProfileType();

    /**
     * @brief Set the ramp type.
     *
     * Currently only a Sin2 (value=1) and a linear/trapezoidal ramp is supported (value = 0)
     * (SDO 0x6086)
     * @param new_motion_profile_type the new motion profile type
     */
    void setMotionProfileType(int16_t new_motion_profile_type);


    /**
     * @brief Set the ramp type.
     *
     * Currently only a Sin2 (value=1) and a linear/trapezoidal ramp is supported (value = 0)
     * (SDO 0x6086)
     * @param new_motion_profile_type the new motion profile type as a CanOpenMotor::union_8_16 struct
     */
    void setMotionProfileType(union_8_16s new_motion_profile_type);


    /**
    * @brief Return the maximum acceleration that may not be exceeded when moving to the end position or
    * the maximum braking deceleration that may not be exceeded when moving to the end position (SDO 0x60C5/6)
    * @param direction 0 for acceleration, 1 for deceleration
    * @return the the maximum acceleration in user-defined units
    */
    uint32_t getMaxAcceleration(uint8_t direction);

    /**
     * @brief Set the maximum acceleration that may not be exceeded when moving to the end position or
     * the maximum braking deceleration that may not be exceeded when moving to the end position
     * @param direction 0 for acceleration, 1 for deceleration
     * @param new_accel the new maximum acceleration
     */
    void setMaxAcceleration(uint8_t direction, uint32_t new_accel);

    /**
     * @brief Set the maximum acceleration that may not be exceeded when moving to the end position or
     * the maximum braking deceleration that may not be exceeded when moving to the end position
     * @param direction 0 for acceleration, 1 for deceleration
     * @param new_accel a CanOpenMotor::union_8_32 containing the new maximum acceleration
     */
    void setMaxAcceleration(uint8_t direction, union_8_32 new_accel);

    /**
     * @brief Return the status word of the motor, that describes the internal state
     * @return CanOpenMotor::status_word
     */
    int16_t getStatus_word() const;

    /**
     * @brief Updates CanOpenMotor::status_word (UNUSED)
     * @param status_word the new value of CanOpenMotor::status_word
     */
    void setStatus_word(unsigned int status_word);

    /**
     * @brief Read the status word from the motor and update CanOpenMotor::status_word
     */
    void readStatus_word();

    /**
     * @brief Set the mode of operation
     * @param mode_ the new target mode for the motor
     */
    void setModeOfOperation(MotorMode mode_);

    /**
     * @brief Read the mode of operation of the motor and update CanOpenMotor::mode
     */
    void readModeOfOperation();

    /**
     * @brief Switch the mode to velocity mode
     * @return true if the switch was successful, false otherwise
     */
    bool enableVelocityMode();

    /**
     * @brief Switch the mode to profile position mode
     * @return true if the switch was successful, false otherwise
     */
    bool enableProfilePositionMode();

    /**
     * @brief Switch the mode to profile torque mode
     * @return true if the switch was successful, false otherwise
     */
    bool enableProfileTorqueMode();

    /**
     * @brief Return the mode of operation
     * @return CanOpenMotor::mode
     */
    CanOpenMotor::MotorMode getModeOfOperation();


    /**
     * @brief Return the maximum following error symmetrically to the demanded position (SDO 0x6065).
     *
     * If the actual position deviates too
     * greatly from the demanded position, a following error is issued.
     * @return the maximum following error symmetrically to the demanded position
     */
    uint32_t getFollowingErrorWindow();

    /**
     * @brief Set the maximum following error symmetrically to the demanded position (SDO 0x6065).
     * @param new_following_window the new value for the following window
     */
    void setFollowingErrorWindow(uint32_t new_following_window);

    /**
     * @brief Set the maximum following error symmetrically to the demanded position (SDO 0x6065).
     * @param new_following_window a CanOpenMotor::union_8_32 struct containing the new value for the following window
     */
    void setFollowingErrorWindow(union_8_32 new_following_window);

    /**
     * @brief Return time in milliseconds until too large a following error leads to an error message (SDO 0x6066).
     * @return time in milliseconds until too large a following error leads to an error message.
     */
    uint16_t getFollowingErrorTimeout();

    /**
     * @brief Set time in milliseconds until too large a following error leads to an error message (SDO 0x6066).
     * @param timeout_ms the new timeout in ms
     */
    void setFollowingErrorTimeout(uint16_t timeout_ms);

    /**
     * @brief Set time in milliseconds until too large a following error leads to an error message (SDO 0x6066).
     * @param timeout_ms a CanOpenMotor::union_8_16 struct containing the new timeout in ms
     */
    void setFollowingErrorTimeout(union_8_16 timeout_ms);

    /**
     * @brief Return a symmetrical range relative to the target position within which the target is considered to be reached (SDO 0x6067).
     * @return the range relative to the target position within which the target is considered to be reached.
     */
    uint32_t getPositionWindow();

    /**
     * @brief Set the symmetrical range relative to the target position within which the target is considered to be reached
     * @param new_position_window the new value for the position window
     */
    void setPositionWindow(uint32_t new_position_window);

    /**
     * @brief Set the symmetrical range relative to the target position within which the target is considered to be reached
     * @param new_position_window a CanOpenMotor::union_8_32 struct containing the new values for the position window
     */
    void setPositionWindow(union_8_32 new_position_window);

    ///TODO 0x6068
    ///Return the position window time (SDO 0x6068). For this time period, the actual position must be within the
    ///position window so that the target position is considered to be reached.
    uint16_t getPositionWindowTime();

    void setPositionWindowTime(uint16_t new_position_window_time);

    void setPositionWindowTime(union_8_16 new_position_window_time);


    /**
     * @brief Return the minimum and maximum position (SDO 0x607B). If this range is exceeded or undercut, an overflow occurs.
     *
     * To prevent this overflow, see also SW_POS_LIMIT (SDO 0x607D).
     * @return a CanOpenMotor::min_max_velocity struct containing the min an max allowed positions
     */
    signed_min_max_velocity getPosRangeLimit();

    void setPosRangeLimit(int32_t min, int32_t max);

    void setPosRangeLimit(signed_min_max_velocity new_limits);

    ///TODO 0x607D
    /**
     * @brief The target position must lie within the limits set here. Before the check, the home offset (SDO 0x607C) is deducted each time:
     *
     * corrected min position limit = min position limit - home offset\n
     * corrected max position limit = max position limit - home offset\n
     * (SDO 0x607D).
     * @return a CanOpenMotor::min_max_velocity struct containing the min an max allowed positions
     */
    signed_min_max_velocity getSWPosLimit();

    void setSWPosLimit(int32_t min, int32_t max);

    void setSWPosLimit(signed_min_max_velocity new_limits);


    ///TODO 0x608F
    /**
     * @brief Return encoder increments per revolution (SDO 0x608F):
     *
     * position encoder resolution = encoder increments / motor revolutions
     * @return the encoder increments per revolution
     */
    fraction_32_32 getPositionEncoderResolution();

    void setPositionEncoderResolution(uint32_t encoder_increments, uint32_t motor_revolutions);

    void setPositionEncoderResolution(fraction_32_32 encoder_settings);

    ///TODO 0x6091
    /**
     * @brief Return number of motor revolutions per revolution of the driving axis:
     *
     * gear ratio = motor shaft revolutions / driving shaft revolutions.
     * @return the encoder increments per revolution
     */
    fraction_32_32 getGearRatio();

    void setGearRatio(uint32_t motor_revolutions, uint32_t shaft_revolutions);

    void setGearRatio(fraction_32_32 new_ratio);

    /******************* HELPER FUNCTIONS **************************/

    /**
     * @brief Run 10 full rotations of the motor to calculate the
     * steps needed for one full rotation
     */
    void calibrate();

    /**
     * @brief Print the current parameters of the motor
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
