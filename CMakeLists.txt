# 设置最低的CMake版本要求
cmake_minimum_required(VERSION 3.10)

# 项目名称和版本
project(Autopicker VERSION 1.0)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)


include_directories(include)

add_library(autopicker SHARED src/robot.cpp)

# 添加可执行文件
add_executable(robot src/main.cpp)
# 链接其他依赖库，如果有需要
file(GLOB LIBRARY_FILES "libs/*.so")
target_link_libraries(robot PUBLIC autopicker ${LIBRARY_FILES})

# 添加可执行文件
add_executable(test src/test.cpp)
target_link_libraries(test PUBLIC autopicker ${LIBRARY_FILES})

 
# set_target_properties(libautopicker PROPERTIES
#     VERSION 1.0
#     SOVERSION 1
#     OUTPUT_NAME "mylib"
# )
 
# target_include_directories(libautopicker PUBLIC include/)
 
# 链接其他依赖库，如果有需要
# target_link_libraries(mylib PUBLIC otherlib)