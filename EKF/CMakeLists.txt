# CMakeLists.txt  
  
# 设置构建类型为 Release  
cmake_minimum_required(VERSION 3.10)  
project(common_kf)  
set(CMAKE_BUILD_TYPE "Debug")
  
set(CMAKE_CXX_STANDARD 14) # 使用 C++14 标准  
  
find_package(PythonLibs REQUIRED)
include_directories(
        ${PYTHON_INCLUDE_DIRS}
)
# 添加当前目录到 include 路径  
include_directories(${PROJECT_SOURCE_DIR}/include)  

# 添加源文件到编译  
add_executable(KalDemo main.cpp)

target_link_libraries(KalDemo
        ${PYTHON_LIBRARIES}
)