cmake_minimum_required(VERSION 3.8)
project(MI_MOTOR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/config.json" DESTINATION "${CMAKE_BINARY_DIR}")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/magic_enum)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
file(GLOB SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_executable(mi_test mi_test.cpp ${SRC} )
add_executable(dm_test dm_test.cpp ${SRC} )
target_link_libraries(mi_test udev libpcanbasic.so magic_enum::magic_enum)
target_link_libraries(dm_test udev libpcanbasic.so magic_enum::magic_enum)