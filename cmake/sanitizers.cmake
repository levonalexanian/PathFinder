# Workspace-wide sanitizer toggles. Injected into every package via
# -DCMAKE_PROJECT_INCLUDE=.../cmake/sanitizers.cmake when colcon invokes CMake.

option(ENABLE_ASAN  "Enable AddressSanitizer"           OFF)
option(ENABLE_TSAN  "Enable ThreadSanitizer"            OFF)
option(ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)

if(ENABLE_ASAN AND ENABLE_TSAN)
  message(FATAL_ERROR "ENABLE_ASAN and ENABLE_TSAN cannot both be ON")
endif()

set(_san_flags "")
if(ENABLE_ASAN)
  list(APPEND _san_flags "-fsanitize=address" "-fno-omit-frame-pointer")
endif()
if(ENABLE_TSAN)
  list(APPEND _san_flags "-fsanitize=thread" "-fno-omit-frame-pointer")
endif()
if(ENABLE_UBSAN)
  list(APPEND _san_flags "-fsanitize=undefined" "-fno-omit-frame-pointer")
endif()

if(_san_flags)
  add_compile_options(${_san_flags})
  add_link_options(${_san_flags})
  message(STATUS "Sanitizers enabled: ${_san_flags}")
endif()
