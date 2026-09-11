cmake_minimum_required(VERSION 3.16)

project(spi_sim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_executable(spi_sim spi_sim.c)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(spi_sim PRIVATE -Wall -Wextra)
elseif(MSVC)
    target_compile_options(spi_sim PRIVATE /W4)
endif()
