cmake_minimum_required(VERSION 3.8)
project(w10_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)
find_package(w10_sim REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(arm_angle_ik src/arm_angle_ik.cpp)
target_link_libraries(arm_angle_ik
  Eigen3::Eigen
  pinocchio::pinocchio
)
target_include_directories(arm_angle_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Build executables
add_executable(ik_test src/ik_test.cpp)
target_link_libraries(ik_test
  arm_angle_ik
  Eigen3::Eigen
  pinocchio::pinocchio
)

# Install library
install(TARGETS arm_angle_ik
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install executables
install(TARGETS ik_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
