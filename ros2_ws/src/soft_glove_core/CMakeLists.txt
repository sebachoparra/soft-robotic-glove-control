cmake_minimum_required(VERSION 3.16)
project(soft_glove_core LANGUAGES C)

add_library(soft_glove_core STATIC
  src/soft_glove_core.c
  src/soft_glove_core_ukf.c
  src/soft_glove_core_ctrl.c
  src/soft_glove_core_sched.c
  src/soft_glove_core_ukf_compose.c)
set_target_properties(soft_glove_core PROPERTIES
  C_STANDARD 11
  C_STANDARD_REQUIRED YES
  C_EXTENSIONS NO)
target_include_directories(soft_glove_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(soft_glove_core PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
# Preserve the validated RP2040 transcription's multidimensional-array qualifier signatures.
set_source_files_properties(src/soft_glove_core_ukf.c PROPERTIES COMPILE_OPTIONS -Wno-pedantic)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  message(STATUS "soft_glove_core: ament_cmake found -- ROS 2 build")
  install(TARGETS soft_glove_core EXPORT export_soft_glove_core
    ARCHIVE DESTINATION lib)
  install(DIRECTORY include/ DESTINATION include)
  ament_export_targets(export_soft_glove_core HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)

  if(BUILD_TESTING)
    enable_language(CXX)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_scaffold_link test/test_scaffold_link.cpp)
    target_link_libraries(test_scaffold_link soft_glove_core)
    find_package(ament_lint_auto REQUIRED)
    # Copyright lint awaits the unresolved repository licensing decision.
    set(ament_cmake_copyright_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()
else()
  message(STATUS "soft_glove_core: ament_cmake NOT found -- standalone build")
endif()
