cmake_minimum_required(VERSION 3.8)
project(decrypt_data)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED PATHS "/usr/include/opencv4")
find_package(cv_bridge REQUIRED)
#find_package(MbedTLS REQUIRED)
find_library(CRYPTOPP_LIB cryptopp)

set(PROJECT_SOURCES
  src/decrypt_data.cpp

  include/decrypt_data.hpp
)

set(PROJECT_SOURCES_DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  ament_index_cpp
  cv_bridge
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(decryption_node ${PROJECT_SOURCES})
ament_target_dependencies(decryption_node ${PROJECT_SOURCES_DEPENDENCIES})

#target_link_libraries(encryption_node
#	PUBLIC MbedTLS::mbedtls
#	       MbedTLS::mbedcrypto
#	       MbedTLS::mbedx509)
target_link_libraries(decryption_node ${CRYPTOPP_LIB})

install(TARGETS 
  decryption_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  include/
  DESTINATION include
)
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}
)

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
