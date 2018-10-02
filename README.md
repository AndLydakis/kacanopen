# inno_can: CANOpen wrapper for 402 compliant devices (motors)

#### Description

This package is a wrapper over [kacanopen](https://kitmedical.github.io/kacanopen/) that exposes CANOpen functionality in a more
intuitive way as well as providing a more flexible ROS interface.

It was tested using the [peak_linux](./inno_can/peak-linux-driver-8.6.0.tar.gz) driver. Follow the [instructions](./inno_can/PCAN-Driver-Linux_UserMan_eng.pdf) to install,
and make sure to use the ```-DDRIVER=peak_linux -DNO_ROS=Off``` flags.

The main addition is the ```CanOpenMotor``` class, that can be instantiated with a ```kaco::master```
and an integer ID. The id should correspond to the id of the device on the CAN bus.

This object can be used to read and write objects to the device without worrying about hex representations and
sub-indexes. These include but are not limited to :

- Target Velocity (set/getTargetVelocity)
- Current Velocity (set/getTargetVelocity)
- Target Position (Relative or Absolute) (setTargetPosition/Relative, getTargetPosition)
- Acceleration/Deceleration/Ramps (<set/get>ProfileAcceleration)
- Position/Velocity error windows (<set/get><Position)
- Encoder Resolution
- Gear Ratios
- Feed Constants
- Switching to different operating modes (Velocity, Profile Position, Profile Torque)
- Setting T/RPDOS through functions

Example initialization:

```cpp
   int main(int argc, char **argv) {
    kaco::Master master;
    const std::string busname = argv[1];
    const std::string baudrate = argv[2];
    const size_t bus_devices = static_cast<const size_t>(std::stoi(argv[3]));
    
    std::cout << "Opening /dev/pcan" << busname << std::endl;
    std::cout << "With baudrate " << baudrate << std::endl;
    std::cout << "Starting Master (involves starting Core)..." << std::endl;
    
    if (!master.start(busname, baudrate)) {
        std::cout << "Starting Master failed." << std::endl;
        return EXIT_FAILURE;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const size_t num_devices = master.num_devices();

    std::cout << num_devices << " devices found on the bus\n";
    assert(num_devices == bus_devices);
    for (size_t i = 0; i < num_devices; ++i) {
        std::cout << "Testing motor " << i + 1 << std::endl;
        std::cout << "Test initializing the device\n";
        auto *motor = new CanOpenMotor(i + 1, std::stoi(baudrate), master);
        assert(motor->getId() == i + 1);
        motor->print();
        delete (motor);
    }
    return EXIT_SUCCESS;
}
```
Please consult the documentation for usage details.

Currently the ROS interface consists of a [node](./inno_can/src/can_ros_publisher.cpp) that detects all motors on the bus,
loops while publishing their current position, and listens for velocity commands.

Motor positions are published as ```std_msgs/Int32``` on ```/can_motor_node/motor_<id>_positions```. To send a target velocity
to a motor publish a vector of size 2 (```std_msgs/Int16MultiArray```) on the ```motor_velocities``` topic. The first element
of the vector should be the motor ID and the second the target velocity in rpm.

To start the node run:
```bash
user@host:~$ rosrun kacanopen can_test_node <path to usb-to-CAN interface> <can bus baudrate> <number of connected devices>
``` 
#### Installation:

Follow the build & installation instructions for the [original KaCanOpen](./README_original.md) stack.

#### Documentation:

Run
```bash
user@host:~$ doxygen Doxyfile
```
to generate documentation in the [doc](./doc) folder

#### Dependencies:
The stack has been tested on Ubuntu 16.04 with the ```peak_linux``` driver on Nanotec motors using 
a Nanotec SMCI36 motor controller.

#### TODO:
- Extend the ROS interface

#### Maintainer
[Andreas Lydakis](andlydakis@gmail.com)
