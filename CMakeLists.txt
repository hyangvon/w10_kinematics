cmake_minimum_required(VERSION 3.8)
project(w10_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Special handling for Eigen3 as it's often header-only
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Include directories
include_directories(include)

# Create kinematics library
ament_auto_add_library(w10_kinematics_lib
  src/w10_kinematics_solver.cpp
)

# Build executables
ament_auto_add_executable(w10_ik_node src/w10_ik_node.cpp)
ament_auto_add_executable(w10_ik_test src/w10_ik_test.cpp)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch and config files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
