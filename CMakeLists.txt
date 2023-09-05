cmake_minimum_required(VERSION 3.5)
project(latency_analysis)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  message_filters
  sensor_msgs
)

add_executable(pub1_exe src/pub1_exe.cpp)
target_include_directories(pub1_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pub1_exe
  "rclcpp"
  "message_filters"
  "sensor_msgs"
  )

add_executable(pub2_exe src/pub2_exe.cpp)
target_include_directories(pub2_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pub2_exe
  "rclcpp"
  "message_filters"
  "sensor_msgs"
  )

add_executable(pub3_exe src/pub3_exe.cpp)
target_include_directories(pub3_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pub3_exe
  "rclcpp"
  "message_filters"
  "sensor_msgs"
  )

  add_executable(pub4_exe src/pub4_exe.cpp)
  target_include_directories(pub4_exe PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  
  ament_target_dependencies(pub4_exe
    "rclcpp"
    "message_filters"
    "sensor_msgs"
    )
  
  add_executable(pub5_exe src/pub5_exe.cpp)
  target_include_directories(pub5_exe PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  
  ament_target_dependencies(pub5_exe
    "rclcpp"
    "message_filters"
    "sensor_msgs"
    )
  
  add_executable(pub6_exe src/pub6_exe.cpp)
  target_include_directories(pub6_exe PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  
  ament_target_dependencies(pub6_exe
    "rclcpp"
    "message_filters"
    "sensor_msgs"
    )

    add_executable(pub7_exe src/pub7_exe.cpp)
    target_include_directories(pub7_exe PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    
    ament_target_dependencies(pub7_exe
      "rclcpp"
      "message_filters"
      "sensor_msgs"
      )
    
    add_executable(pub8_exe src/pub8_exe.cpp)
    target_include_directories(pub8_exe PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    
    ament_target_dependencies(pub8_exe
      "rclcpp"
      "message_filters"
      "sensor_msgs"
      )
    
    add_executable(pub9_exe src/pub9_exe.cpp)
    target_include_directories(pub9_exe PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    
    ament_target_dependencies(pub9_exe
      "rclcpp"
      "message_filters"
      "sensor_msgs"
      )

add_executable(sub_exe src/sub_exe.cpp)
target_include_directories(sub_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(sub_exe
  "rclcpp"
  "message_filters"
  "sensor_msgs"
  )

install(TARGETS 
  pub1_exe
  pub2_exe
  pub3_exe
  pub4_exe
  pub5_exe
  pub6_exe
  pub7_exe
  pub8_exe
  pub9_exe
  sub_exe
  DESTINATION lib/${PROJECT_NAME})


# Install files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
