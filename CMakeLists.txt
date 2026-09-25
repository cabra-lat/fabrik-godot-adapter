cmake_minimum_required(VERSION 3.24)
project(fabrik_godot_adapter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(FABRIK_CORE_ROOT "" CACHE PATH "Install prefix containing include/fabrik_core.h and lib/libfabrik_core.a")
set(GODOT_CPP_SOURCE_DIR "" CACHE PATH "Checked-out godot-cpp source tree (use the 4.4-stable tag for Godot 4.x ABI compatibility)")

if(NOT FABRIK_CORE_ROOT)
  message(FATAL_ERROR "Set FABRIK_CORE_ROOT to a fabrik_core install prefix")
endif()
find_path(FABRIK_CORE_INCLUDE_DIR fabrik_core.h
  PATHS "${FABRIK_CORE_ROOT}/include" REQUIRED)
find_library(FABRIK_CORE_LIBRARY NAMES fabrik_core
  PATHS "${FABRIK_CORE_ROOT}/lib" REQUIRED)
find_program(FABRIK_FORTRAN_EXECUTABLE NAMES gfortran REQUIRED)
if(WIN32)
  set(FABRIK_GFORTRAN_SO libgfortran.dll.a)
elseif(APPLE)
  set(FABRIK_GFORTRAN_SO libgfortran.dylib)
else()
  set(FABRIK_GFORTRAN_SO libgfortran.so)
endif()
execute_process(
  COMMAND "${FABRIK_FORTRAN_EXECUTABLE}" -print-file-name=${FABRIK_GFORTRAN_SO}
  OUTPUT_VARIABLE FABRIK_GFORTRAN_LIBRARY
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT EXISTS "${FABRIK_GFORTRAN_LIBRARY}")
  message(FATAL_ERROR "Could not locate libgfortran.so from ${FABRIK_FORTRAN_EXECUTABLE}")
endif()
if(NOT GODOT_CPP_SOURCE_DIR)
  message(FATAL_ERROR "Set GODOT_CPP_SOURCE_DIR to a godot-cpp source checkout; no opaque binaries are vendored")
endif()

# godot-cpp's own CMake project supplies generated headers and bindings.
add_subdirectory("${GODOT_CPP_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/godot-cpp" EXCLUDE_FROM_ALL)
if(TARGET godot-cpp::template_debug)
  set(FABRIK_GODOT_CPP_TARGET godot-cpp::template_debug)
elseif(TARGET godot-cpp)
  set(FABRIK_GODOT_CPP_TARGET godot-cpp)
else()
  message(FATAL_ERROR "The selected godot-cpp source tree exposes no supported CMake target")
endif()

add_library(fabrik_adapter SHARED
  src/register_types.cpp
  src/fabrik_chain_3d.cpp
)
target_include_directories(fabrik_adapter PRIVATE
  "${FABRIK_CORE_INCLUDE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(fabrik_adapter PRIVATE
  "${FABRIK_CORE_LIBRARY}"
  "${FABRIK_GFORTRAN_LIBRARY}"
  "${FABRIK_GODOT_CPP_TARGET}"
)
if(UNIX AND NOT APPLE)
  target_link_options(fabrik_adapter PRIVATE "-Wl,--no-undefined")
endif()

add_executable(fabrik_cxx_smoke tests/fabrik_cxx_smoke.cpp)
target_include_directories(fabrik_cxx_smoke PRIVATE "${FABRIK_CORE_INCLUDE_DIR}")
target_link_libraries(fabrik_cxx_smoke PRIVATE
  "${FABRIK_CORE_LIBRARY}"
  "${FABRIK_GFORTRAN_LIBRARY}"
)
target_compile_features(fabrik_cxx_smoke PRIVATE cxx_std_17)

enable_testing()
add_test(NAME fabrik_cxx_smoke COMMAND fabrik_cxx_smoke)

set_target_properties(fabrik_adapter PROPERTIES
  PREFIX "lib"
  OUTPUT_NAME "fabrik_adapter"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
# Keep the descriptor beside the source-tree demo project. CI copies only the
# freshly built library into bin/; no binary is part of the source package.
