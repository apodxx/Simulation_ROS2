cmake_minimum_required(VERSION 3.8)
project(xuegecar_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# launch/urdf/rviz/meshes/worlds 都是仿真和可视化需要的资源
install(
  DIRECTORY launch meshes rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# model.config 让 Gazebo 能用 model://xuegecar_description/meshes/xxx.stl 找到网格
install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
