cmake_minimum_required(VERSION 3.16)
project(aero_cfd_wasm CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_executable(lbm_smoke src/native_main.cpp)
target_include_directories(lbm_smoke PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_compile_options(lbm_smoke PRIVATE -O3 -Wall -Wextra)

enable_testing()
add_test(NAME smoke COMMAND lbm_smoke)
set_tests_properties(smoke PROPERTIES FAIL_REGULAR_EXPRESSION "RESULT     = FAIL")
