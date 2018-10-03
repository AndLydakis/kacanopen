//
// Created by lydakis on 05/09/18.
//
#include <cstdlib>
#include <CanOpenMotorROS.h>
#include <CanOpenMotor.h>
#include <thread>
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt32.h"

std::vector<CanOpenMotor> motors{};
std::vector<ros::Publisher> publishers{};

void velocityCallback(const std_msgs::Int16MultiArray::ConstPtr &msg) {
    try {
        if (msg->layout.dim[0].size < 2) {
            ROS_ERROR("Velocity message has wrong dimensions");
            return;
        }
        ROS_INFO("Velocity callback for motor %d with velocity: %d", msg->data[0], msg->data[1]);
        motors[msg->data[0] - 1].setTargetVelocity(msg->data[1]);

    } catch (kaco::canopen_error &ce) {
        ROS_ERROR("CAN Exception: %s", ce.what());
    } catch (ros::Exception &re) {
        ROS_ERROR("ROS Exception: %s", re.what());
    }
}

void MotorSpin(CanOpenMotor &motor) {
    ros::Rate r(50);
    while (ros::ok()) {
        ROS_ERROR("Spinning motor %lu", motor.getId());
        std_msgs::UInt32 position_msg;
        position_msg.data = static_cast<unsigned int>(motor.getCurrentPosition());
        publishers[motor.getId() - 1].publish(position_msg);
        ros::spinOnce();
        r.sleep();
    }
}

int main(int argc, char **argv) {
    std::string busname = argv[1];
    std::string baudrate = argv[2];
    size_t bus_devices = static_cast<const size_t>(std::stoi(argv[3]));

    ros::init(argc, argv, "can_motor_node");
    ros::NodeHandle nh("~");

    std::cout << "Opening /dev/pcan" << busname << std::endl;
    std::cout << "With " << bus_devices << " devices\n";
    std::cout << "With baudrate " << baudrate << std::endl;
    kaco::Master master;
    master.start(busname, baudrate);
    assert(master.num_devices() == bus_devices);

    ros::Subscriber sub = nh.subscribe<std_msgs::Int16MultiArray>("/motor_velocities", 10, velocityCallback);

    std::vector<std::thread> motor_threads{};

    for (size_t i = 0; i < bus_devices; ++i) {
        publishers.emplace_back(nh.advertise<std_msgs::UInt32>("motor_" + std::to_string(i + 1) + "_position", 10));
        motors.emplace_back(CanOpenMotor(i + 1, 1000, master));
        motors.back().DisableOperation();
        usleep(1000000);
        motors.back().enableVelocityMode();
        usleep(1000000);
        motors.back().EnableOperation();
        usleep(1000000);
    }
    for (size_t i = 0; i < bus_devices; ++i) {
        motor_threads.emplace_back(std::thread(MotorSpin, std::ref(motors[i])));
    }

    ros::Rate rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    for (size_t i = 0; i < bus_devices; ++i) {
        motor_threads[i].join();
    }
    return EXIT_SUCCESS;
}

//
/**
rostopic pub /motor_velocities std_msgs/Int16MultiArray "layout:
dim:
- label: ''
size: 2
stride: 0
data_offset: 0
data: [1, 3000]"

 **/