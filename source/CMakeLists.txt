include_directories(${Quadson_SOURCE_DIR}/include)
include_directories(${Quadson_SOURCE_DIR}/library/Navio)
# SET(ENV{LD_LIBRARY_PATH} "/usr/local/lib:$ENV{LD_LIBRARY_PATH}")
set(THREADS_PREFER_PTHREAD_FLAG ON)

add_library(libnavio2 SHARED IMPORTED)
set_property(TARGET libnavio2 PROPERTY IMPORTED_LOCATION "${Quadson_SOURCE_DIR}/library/Navio/build/libnavio2.so")
# add_library(libnavio2 /usr/local/lib)
find_package(Threads REQUIRED)
# I assume you want to use LibProject as a library in MainProject.
add_executable( quadson main.cpp
                        quadson.cpp
                        can_interface.cpp
                        actuator.cpp
                        leg_group.cpp
                        AHRS.cpp )
target_link_libraries(quadson PRIVATE Threads::Threads libnavio2)
