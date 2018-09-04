//
// Created by lydakis on 03/09/18.
//

#define BOOST_TEST_MODULE can_open_tests

#include <boost/test/included/unit_test.hpp>
#include <CanOpenMotor.h>

namespace utf = boost::unit_test;


struct ArgsFixture {
    ArgsFixture() : argc(boost::unit_test::framework::master_test_suite().argc),
                    argv(boost::unit_test::framework::master_test_suite().argv) {
        busname = argv[1];
        baudrate = argv[2];
        bus_devices = static_cast<const size_t>(std::stoi(argv[3]));
        std::cout << "Opening /dev/pcan" << busname << std::endl;
        std::cout << "With baudrate " << baudrate << std::endl;
        BOOST_REQUIRE(master.start(busname, baudrate) > 0);
        BOOST_REQUIRE(master.num_devices() == bus_devices);
        for (size_t i = 0; i < bus_devices; ++i) {
            motors.emplace_back(new CanOpenMotor(i + 1, std::stoi(baudrate), master));
            BOOST_REQUIRE(motors[i]->getId() == i + 1);
        }
    }

    int argc;
    char **argv;
    kaco::Master master;
    std::string busname;
    std::string baudrate;
    size_t bus_devices;
    std::vector<CanOpenMotor *> motors;
};


BOOST_FIXTURE_TEST_CASE(test_switch_to_profile_position_mode, ArgsFixture) {
    for (size_t i = 0; i < bus_devices; ++i) {}
}

BOOST_FIXTURE_TEST_CASE(test_switch_to_velocity_mode, ArgsFixture) {
    BOOST_REQUIRE(1 == 1);
}