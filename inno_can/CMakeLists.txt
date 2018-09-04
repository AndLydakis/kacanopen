cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

project(inno_can)

message(STATUS "Configuring Inno Can packages")

include_directories(src)
include_directories(include)



add_executable(ticks_test_node src/ticks_test.cpp ../inno_can/include/CanOpenMotor.h ../inno_can/src/CanOpenMotor.cpp)
target_link_libraries(ticks_test_node kacanopen_core)
target_link_libraries(ticks_test_node kacanopen_master)
target_link_libraries(ticks_test_node kacanopen_ros_bridge)


add_executable(can_test_node src/can_test.cpp ../inno_can/include/CanOpenMotor.h ../inno_can/src/CanOpenMotor.cpp)
target_link_libraries(can_test_node kacanopen_core)
target_link_libraries(can_test_node kacanopen_master)
target_link_libraries(can_test_node kacanopen_ros_bridge)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost COMPONENTS unit_test_framework REQUIRED)
add_executable(simple-test test/tests.cpp ../inno_can/include/CanOpenMotor.h ../inno_can/src/CanOpenMotor.cpp)
target_link_libraries(simple-test ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_link_libraries(simple-test kacanopen_core)
target_link_libraries(simple-test kacanopen_master)
target_link_libraries(simple-test kacanopen_ros_bridge)
#add_executable(can_motor_tests test/tests_main.cpp test/tests.cpp  ../inno_can/include/catch.hpp)
#target_link_libraries(can_motor_tests kacanopen_core)
#target_link_libraries(can_motor_tests kacanopen_master)
#target_link_libraries(can_motor_tests kacanopen_ros_bridge)


#foreach(TARGET_SRC ${TARGETS_SRC})
#
#    get_filename_component(TARGET_NAME ${TARGET_SRC} NAME_WE)
#    set(TARGET_NAME "kacanopen_example_${TARGET_NAME}")
#    add_executable(${TARGET_NAME} ${TARGET_SRC})
#    set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 11)
#    target_link_libraries(${TARGET_NAME} kacanopen_core)
#    target_link_libraries(${TARGET_NAME} kacanopen_master)
#
#    if (NOT(${NO_ROS}))
#        target_link_libraries(${TARGET_NAME} kacanopen_ros_bridge)
#    endif()
#
#    list(APPEND KACANOPEN_EXAMPLES_TARGET_NAMES ${TARGET_NAME})
#
#endforeach(TARGET_SRC ${TARGETS_SRC})
#
## Install
#if (${INSTALL_EXAMPLES})
#    kacanopen_install("${KACANOPEN_EXAMPLES_TARGET_NAMES}" FALSE)
#endif()
#
