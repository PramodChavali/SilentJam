cmake_minimum_required(VERSION 3.20)
project(trumpet_synth)

set(CMAKE_CXX_STANDARD 17)

find_package(aubio CONFIG REQUIRED)
find_package(portaudio CONFIG REQUIRED)

add_executable(trumpet_synth src/trumpet_synth.cpp)

target_include_directories(trumpet_synth PRIVATE
    ${CMAKE_SOURCE_DIR}/build/vcpkg_installed/x64-windows/include
)

target_link_directories(trumpet_synth PRIVATE
    ${CMAKE_SOURCE_DIR}/build/vcpkg_installed/x64-windows/lib
)

target_link_libraries(trumpet_synth PRIVATE aubio portaudio)
