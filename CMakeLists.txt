cmake_minimum_required(VERSION 3.11)
project(A2PAA)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

add_executable(A2PAA main.cpp
        classes/CityGraph.cpp
        classes/CityGraph.h)
target_link_libraries(A2PAA)

add_executable(A2PAA_chart main_chart.cpp
        classes/CityGraph.cpp
        classes/CityGraph.h
        tests/charts.cpp
        tests/charts.h)

target_link_libraries(A2PAA)

enable_testing()
add_executable(
        A2PAA_test
        tests/test.cpp
        classes/CityGraph.cpp
        classes/CityGraph.h
)
target_link_libraries(A2PAA_test gtest_main gtest)

include(GoogleTest)
gtest_discover_tests(A2PAA_test)


