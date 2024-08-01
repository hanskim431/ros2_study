cmake_minimum_required(VERSION 3.8)
project(count_book_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(count_book_cpp REQUIRED)

set(srv_files
  "srv/CountBook.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

add_executable(server src/server_count_book.cpp)
ament_target_dependencies(server rclcpp count_book_cpp)

add_executable(client src/client_count_book.cpp)
ament_target_dependencies(client rclcpp count_book_cpp)

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
