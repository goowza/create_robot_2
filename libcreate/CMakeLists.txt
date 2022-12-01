cmake_minimum_required(VERSION 3.8)
project(libcreate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Threads REQUIRED)

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

set(LIBRARY_NAME libcreate)

# Specify locations of header files
include_directories(
  include
)

# Declare cpp library
add_library(${LIBRARY_NAME} SHARED
  src/create.cpp
  src/serial.cpp
  src/serial_stream.cpp
  src/serial_query.cpp
  src/data.cpp
  src/packet.cpp
  src/types.cpp
)

ament_export_targets(${LIBRARY_NAME} HAS_LIBRARY_TARGET)

# Manually link to thread library for build on ARM
if(THREADS_HAVE_PTHREAD_ARG)
  set_property(TARGET ${LIBRARY_NAME} PROPERTY COMPILE_OPTIONS "-pthread")
  set_property(TARGET ${LIBRARY_NAME} PROPERTY INTERFACE_COMPILE_OPTIONS "-pthread")
endif()

if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(${LIBRARY_NAME} "${CMAKE_THREAD_LIBS_INIT}")
endif()

# Link to Boost
target_link_libraries(${LIBRARY_NAME}
  ${Boost_LIBRARIES}
)

set(LIB_INSTALL_DIR "lib")
set(BIN_INSTALL_DIR "bin")
set(INCLUDE_INSTALL_DIR "include")

# Install targets
install(
  TARGETS ${LIBRARY_NAME}
  EXPORT ${LIBRARY_NAME}
  LIBRARY DESTINATION "${LIB_INSTALL_DIR}"
  ARCHIVE DESTINATION "${LIB_INSTALL_DIR}"
  RUNTIME DESTINATION "${BIN_INSTALL_DIR}"
  INCLUDES DESTINATION "${INCLUDE_INSTALL_DIR}"
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION ${INCLUDE_INSTALL_DIR}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
