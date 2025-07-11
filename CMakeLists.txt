# 设置最低的CMake版本要求
cmake_minimum_required(VERSION 3.10)

# 项目名称和版本
project(Autopicker VERSION 1.0)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0") # 启用调试符号，禁用优化；单行调试


include_directories(include)

add_library(autopicker SHARED src/robot.cpp)

# 添加可执行文件
add_executable(robot src/main.cpp src/robot.cpp)
# 链接其他依赖库，如果有需要
file(GLOB LIBRARY_FILES "libs/*.so")
target_link_libraries(robot PUBLIC autopicker ${LIBRARY_FILES} -lpthread -lm)

# 添加 nlohmann/json 库
find_package(nlohmann_json REQUIRED)
target_link_libraries(autopicker PRIVATE nlohmann_json::nlohmann_json)

# 添加可执行文件
# add_executable(test src/test.cpp)
# target_link_libraries(test PUBLIC autopicker ${LIBRARY_FILES})

 
# set_target_properties(libautopicker PROPERTIES
#     VERSION 1.0
#     SOVERSION 1
#     OUTPUT_NAME "mylib"
# )
 
# target_include_directories(libautopicker PUBLIC include/)
 
# 链接其他依赖库，如果有需要
# target_link_libraries(mylib PUBLIC otherlib)