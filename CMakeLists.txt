cmake_minimum_required(VERSION 3.16)
project(hnes)


set(CMAKE_C_STANDARD 17)

include_directories("include")

# Uncomment to enable memory checker. Doesn't work while using GDB.
#add_compile_options(-fsanitize=address)
#add_link_options(-fsanitize=address)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -ggdb")

add_executable(hnes
    src/main.c
    src/rom.c
)
