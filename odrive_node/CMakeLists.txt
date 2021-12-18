cmake_minimum_required(VERSION 3.5)
project(odrive_node)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
        include
)

set(dependencies
        "rclcpp"
        "std_msgs"
        "geometry_msgs"
        )

set(EXEC_NAME "odrive_node")

add_executable(${EXEC_NAME}
        src/odrive_node.cpp
        src/ODrive.cpp
        )

ament_target_dependencies(${EXEC_NAME} ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY include/
        DESTINATION include/
        )

################################################################################
# Macro for ament package
################################################################################
ament_package()
