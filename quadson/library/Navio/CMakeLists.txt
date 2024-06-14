cmake_minimum_required(VERSION 2.4.0)

project(navio2)

# Find source files
file(GLOB SOURCES Navio2/*.cpp Common/*.cpp)

# Include header files
# include_directories(Navio2)
include_directories(${CMAKE_CURRENT_LIST_DIR})

# Create shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Install library
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION lib ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION bin ${CMAKE_INSTALL_BINDIR})

# Install library headers
file(GLOB HEADERS Navio2/*.h)
install(FILES ${HEADERS} DESTINATION include/Navio2)

file(GLOB HEADERSCommon Common/*.h)
install(FILES ${HEADERSCommon} DESTINATION include/Common)