cmake_minimum_required(VERSION 3.8)
project(daheng_camera_driver)

# 使用C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# 添加可执行文件
add_executable(image_publisher
  src/daheng_camera.cpp
  src/image_publisher.cpp
)

# 包含目录
target_include_directories(image_publisher PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(image_publisher
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  image_transport::image_transport
  libgxiapi.so  # 大恒SDK库
)

# 安装
install(TARGETS image_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()