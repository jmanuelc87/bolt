cmake_minimum_required(VERSION 3.14)
project(bolt_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(bolt_tests
    test_frames.cpp
    test_parser.cpp
    test_visitor.cpp
    test_flash_controller.cpp
)

target_include_directories(bolt_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${CMAKE_CURRENT_SOURCE_DIR}/../Bolt/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../Bolt/Inc/bolt
)

target_link_libraries(bolt_tests PRIVATE
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(bolt_tests)
