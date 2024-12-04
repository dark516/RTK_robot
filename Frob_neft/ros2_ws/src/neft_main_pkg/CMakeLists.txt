cmake_minimum_required(VERSION 3.5)
project(neft_main_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install the launch files
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

