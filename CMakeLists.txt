cmake_minimum_required(VERSION 3.8)
project(ariac_collection_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# declares a dependency on GoogleTest which is downloaded from GitHub
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Enables testing for this directory and below
enable_testing()
include(GoogleTest)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ariac_msgs REQUIRED)	
find_package(std_srvs REQUIRED)
find_package(shape_msgs REQUIRED)	
find_package(moveit_ros_planning_interface REQUIRED)

include_directories(include/ariac_collection_bot)

set(SENSOR_CAMERA_DEPENDENCIES
  rclcpp
  std_srvs
  ariac_msgs
  geometry_msgs
  tf2  
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
)

foreach(Dependency IN ITEMS ${SENSOR_CAMERA_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(ccs src/ccs.cpp src/ccs_main.cpp)			     
ament_target_dependencies(ccs rclcpp ariac_msgs std_srvs shape_msgs moveit_ros_planning_interface)

install(TARGETS				
	ccs			
	DESTINATION lib/${PROJECT_NAME}		
	)

install(DIRECTORY include
	launch
	config
	rviz
	DESTINATION share/${PROJECT_NAME})	

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	#set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	#set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
