cmake_minimum_required(VERSION 3.8)
project(serial)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

set(serial_sources src/serial.cc)

if(WIN32)
  list(APPEND serial_sources
    src/impl/win.cc
    src/impl/list_ports/list_ports_win.cc)
elseif(APPLE)
  list(APPEND serial_sources
    src/impl/unix.cc
    src/impl/list_ports/list_ports_osx.cc)
else()
  list(APPEND serial_sources
    src/impl/unix.cc
    src/impl/list_ports/list_ports_linux.cc)
endif()

add_library(${PROJECT_NAME} SHARED ${serial_sources})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(WIN32)
  target_link_libraries(${PROJECT_NAME} PRIVATE setupapi)
elseif(APPLE)
  find_library(IOKIT_LIBRARY IOKit REQUIRED)
  find_library(FOUNDATION_LIBRARY Foundation REQUIRED)
  target_link_libraries(${PROJECT_NAME} PRIVATE
    ${IOKIT_LIBRARY} ${FOUNDATION_LIBRARY} Threads::Threads)
else()
  target_link_libraries(${PROJECT_NAME} PRIVATE rt Threads::Threads)
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
