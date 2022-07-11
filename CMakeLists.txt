cmake_minimum_required(VERSION 3.5)
project(virtual_can)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(imperial_driverless_interfaces REQUIRED)

# Make FS-AI_API
add_custom_target(fs-ai_api ALL
    COMMAND $(MAKE) -C ${PROJECT_SOURCE_DIR}/FS-AI_API/FS-AI_API -f makefile
)

# Create virtual_can_node executable
include_directories(include FS-AI_API/FS-AI_API)
add_executable(virtual_can_node src/virtual_can.cpp)
add_dependencies(virtual_can_node fs-ai_api)
target_link_libraries(virtual_can_node ${PROJECT_SOURCE_DIR}/FS-AI_API/FS-AI_API/fs-ai_api.a)

ament_target_dependencies(virtual_can_node rclcpp sensor_msgs imperial_driverless_interfaces)

# Install executable
install(TARGETS virtual_can_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
