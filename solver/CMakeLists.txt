cmake_minimum_required(VERSION 3.10)
project(solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

enable_language(CUDA)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDAToolkit REQUIRED)

set(T_INFER_ROOT "/home/zst/T/src/tdt_vision/infer")
set(T_INFER_BUILD "/home/zst/T/src/build/tdt_vision/infer")

if(NOT EXISTS "${T_INFER_ROOT}/include/yolos.hpp")
  message(FATAL_ERROR "T infer headers not found: ${T_INFER_ROOT}/include")
endif()

if(NOT EXISTS "${T_INFER_BUILD}/libinfer.a")
  message(FATAL_ERROR "T infer library not found: ${T_INFER_BUILD}/libinfer.a")
endif()

add_executable(solver_node
  src/solver_node.cpp
  src/trt_yolo_bridge.cu
)
ament_target_dependencies(solver_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  interfaces
)
target_include_directories(solver_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${T_INFER_ROOT}/include
)
target_compile_definitions(solver_node PRIVATE FYT_ENABLE_TRT=1)
target_compile_options(solver_node PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-Wall -Werror -O2>
  $<$<COMPILE_LANGUAGE:CUDA>:-O2>
)
set_target_properties(solver_node PROPERTIES CUDA_ARCHITECTURES "86")
target_link_libraries(solver_node
  ${OpenCV_LIBS}
  ${T_INFER_BUILD}/libinfer.a
  nvinfer
  nvinfer_plugin
  nvonnxparser
  CUDA::cudart
  CUDA::cublas
)

add_library(lt_control STATIC
  src/lt_controller.cpp
)
target_include_directories(lt_control PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(lt_control
  ${OpenCV_LIBS}
)
target_compile_options(lt_control PRIVATE -Wall -Werror -O2)

target_include_directories(solver_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(solver_node
  lt_control
)

add_library(gimbal_serial_compat STATIC
  src/gimbal_serial_protocol.cpp
  src/gimbal_serial_port.cpp
)
target_include_directories(gimbal_serial_compat PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(gimbal_serial_compat PRIVATE -Wall -Werror -O2)

add_executable(dji_c_serial_bridge_node
  src/dji_c_serial_bridge_node.cpp
)
ament_target_dependencies(dji_c_serial_bridge_node
  rclcpp
  std_msgs
  interfaces
)
target_compile_options(dji_c_serial_bridge_node PRIVATE -Wall -Werror -O2)

add_executable(gimbal_serial_compat_node
  src/gimbal_serial_compat_node.cpp
)
target_include_directories(gimbal_serial_compat_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(gimbal_serial_compat_node
  rclcpp
  interfaces
)
target_link_libraries(gimbal_serial_compat_node
  gimbal_serial_compat
)
target_compile_options(gimbal_serial_compat_node PRIVATE -Wall -Werror -O2)

install(TARGETS
  solver_node
  dji_c_serial_bridge_node
  gimbal_serial_compat_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
