/*
 * Copyright (c) 2016, Thomas Keh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <thread>
#include <chrono>
#include <cstdint>

#include "master.h"
#include "canopen_error.h"
#include "logger.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

int vel = 0;

void joy_callback(const sensor_msgs::JoyConstPtr& msg){
    vel = msg->axes[4] * 1000;
}

int main(int argc, char** argv) {
    
    if (argc < 3){
        std::cout << "Too few arguments, exiting\n";
        return EXIT_FAILURE;
    }
    // ----------- //
    // Preferences //
    // ----------- //

    // The node ID of the slave we want to communicate with.
    const uint8_t node_id = 1;

    // Set the name of your CAN bus. "slcan0" is a common bus name
    // for the first SocketCAN device on a Linux system.
    const std::string busname = argv[1];

    // Set the baudrate of your CAN bus. Most drivers support the values
    // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
    const std::string baudrate = argv[2];

    // -------------- //
    // Initialization //
    // -------------- //

    std::cout << "This is an example which shows the usage of the Master library." << std::endl;

    // Create Master (includes Core - accessible via master.core).
    kaco::Master master;

    std::cout << "Starting Master (involves starting Core)..." << std::endl;
    if (!master.start(busname, baudrate)) {
        std::cout << "Starting Master failed." << std::endl;
        return EXIT_FAILURE;
    }

    bool found_device = false;
    size_t device_index;

    while (!found_device) {

        for (size_t i = 0; i < master.num_devices(); ++i) {
            kaco::Device &device = master.get_device(i);
            if (device.get_node_id() == node_id) {
                found_device = true;
                device_index = i;
                break;
            }
        }

        std::cout << "Device with ID " << (unsigned) node_id
                  << " has not been found yet. Waiting one more second. Press Ctrl+C abort." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

    }

    // ------------ //
    // Device usage //
    // ------------ //

    kaco::Device &device = master.get_device(device_index);

    std::cout << "Starting device with ID " << (unsigned) node_id << "..." << std::endl;
    device.start();

    std::cout << "Loading object dictionary from the library. This can be either a generic CiA-301"
              << " dictionary, a CiA-profile specific dictionary, or a manufacturer./device-specific dictionary."
              << std::endl;
    const std::string loaded_eds_file = device.load_dictionary_from_library();
    std::cout << "Loaded the following EDS file from the library: " << loaded_eds_file << std::endl;

    std::cout << "Alternatively, you could load your own EDS file via device.load_dictionary_from_eds(path)."
              << std::endl;
    // device.load_dictionary_from_eds("...");

    std::cout << "The following should work for all CANopen-compliant devices." << std::endl;

    std::cout << "CiA-profile = " << device.get_device_profile_number() << std::endl;

    std::cout << "Vendor-ID = " << device.get_entry("Identity object/Vendor-ID") << std::endl;

    std::cout << "The following works for most CANopen-compliant devices (however, the entries are not mandatory)."
              << std::endl;
    try {

        std::cout << "Manufacturer device name = " << device.get_entry("Manufacturer device name") << std::endl;

        std::cout << "Manufacturer hardware version = " << device.get_entry("Manufacturer hardware version")
                  << std::endl;

        std::cout << "Manufacturer software version = " << device.get_entry("Manufacturer software version")
                  << std::endl;

    } catch (const kaco::canopen_error &error) {
        std::cout << "Getting manufacturer information failed: " << error.what() << std::endl;
    }
    
    device.read_complete_dictionary();
    device.print_dictionary();
    device.print_constants();
    device.print_operations();
   
    device.add_receive_pdo_mapping(0x181, "vl_target_velocity", 0);
    //device.add_receive_pdo_mapping(0x181, "vl_target_velocity_actual", 2);

    device.add_receive_pdo_mapping(0x281, "position_actual_value", 0);
    device.add_receive_pdo_mapping(0x281, "position_actual_value*", 3);

    device.add_receive_pdo_mapping(0x318, "statusword", 0);
    
    device.add_transmit_pdo_mapping(0x201, {{"vl_target_velocity", 0}});

    device.set_entry("modes_of_operation", device.get_constant("velocity_mode"));
    //device.execute("set_controlword_flag", "controlword_enable_voltage");
    device.execute("enable_operation");
    device.execute("set_controlword_flag", "controlword_halt");
    device.set_entry("vl_target_velocity", (int16_t) 0);
    device.execute("unset_controlword_flag", "controlword_halt");
    ros::init(argc, argv, "canopen_test_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("joy", 10, joy_callback);
    int counter = 0;
    while (ros::ok()) {
        //ROS_ERROR("%d", counter);
        try {
	    //device.execute("set_target_velocity", (int16_t) vel);
	    device.set_entry("vl_target_velocity", (int16_t) vel, kaco::WriteAccessMethod::cache);
            counter ++;
            counter = counter%100;
            if (counter == 0){
               std::cout << "Reading target velocity: " << device.get_entry("vl_target_velocity", kaco::ReadAccessMethod::cache) << std::endl;
	        std::cout << "Reading motor position: " << device.get_entry("position_actual_value", kaco::ReadAccessMethod::cache)  << std::endl;
		std::cout << "Reading internal motor position: " << device.get_entry("position_actual_value*", kaco::ReadAccessMethod::cache) << std::endl;
                std::cout << "Status: " << device.get_entry("statusword") << std::endl;
            }
            ros::spinOnce();
        } catch (const kaco::canopen_error &error) {
            std::cout << "KaCanOpen Error: " << error.what() << std::endl;
        }
    }
}
