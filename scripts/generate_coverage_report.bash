#!/bin/bash
#
# This script should be invoked by "ros2 run" and after the unit test
# has been run.
#


set -ue

PROG_DIR=$(dirname $(readlink -f "$0"))
EXEC_DIR=$(pwd -P)
PROG_NAME=$(basename "$0")
ROS_PACKAGE_NAME=$(basename $PROG_DIR)

echo "Code location: $PROG_DIR"
echo "Executing from: $EXEC_DIR"
echo "ROS Package Name: $ROS_PACKAGE_NAME"

# Define the directory for coverage files
COVERAGE_DIR=$EXEC_DIR/install/ariac_collection_bot/lib/ariac_collection_bot

# 1. Generate coverage report info
rm -f $COVERAGE_DIR/coverage.info
lcov --capture --directory $EXEC_DIR/build/ariac_collection_bot/CMakeFiles/test_ariac_collection_bot.dir/test/ --output-file $COVERAGE_DIR/coverage.info

# 2. Exclude some files from the report
lcov --remove $COVERAGE_DIR/coverage.info \
     '/opt/*' \
     '/usr/*' \
     '*rclcpp/*' \
     '*libstatistics_collector/*' \
     '*rosidl_runtime*' \
     '*rcl_interfaces*' \
     '*rmw/rmw/*' \
     '*tracetools/*' \
     '*_msgs/*' \
     '*/gtest*' \
     'gtest/*' \
     --output-file $COVERAGE_DIR/coverage_cleaned.info

# 3. Finally generate the coverage report
rm -rf $EXEC_DIR/install/ariac_collection_bot/coverage/
genhtml $COVERAGE_DIR/coverage_cleaned.info --output-directory $EXEC_DIR/install/ariac_collection_bot/coverage

echo "Code Coverage generated:"
echo "     $COVERAGE_DIR/coverage_cleaned.info"
echo "     $EXEC_DIR/install/ariac_collection_bot/coverage/index.html"

