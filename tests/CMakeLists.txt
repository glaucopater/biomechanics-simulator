# Smoke test: link against Jolt and run a minimal check
add_executable(smoke_test smoke_test.cpp)
target_include_directories(smoke_test PRIVATE ${PROJECT_SOURCE_DIR}/include ${joltphysics_SOURCE_DIR}/Jolt)
target_link_libraries(smoke_test PRIVATE Jolt)
if(MSVC)
  set_property(TARGET smoke_test PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
  target_compile_options(smoke_test PRIVATE $<$<CONFIG:Release>:/MD> $<$<CONFIG:Debug>:/MDd>)
  target_link_options(smoke_test PRIVATE $<$<CONFIG:Release>:/NODEFAULTLIB:LIBCMT> $<$<CONFIG:Debug>:/NODEFAULTLIB:LIBCMTd>)
endif()
add_test(NAME SmokeTest COMMAND smoke_test)

# E2E verification: full scene, standing + walking stability, drawable body count (no GL)
set(E2E_SOURCES
  e2e_verification.cpp
  ${PROJECT_SOURCE_DIR}/src/Log.cpp
  ${PROJECT_SOURCE_DIR}/src/HumanRagdoll.cpp
  ${PROJECT_SOURCE_DIR}/src/SimulatorScene.cpp
  ${PROJECT_SOURCE_DIR}/src/PoseController.cpp
  ${PROJECT_SOURCE_DIR}/src/DrawableBodyCount.cpp
)
add_executable(e2e_verification ${E2E_SOURCES})
target_include_directories(e2e_verification PRIVATE ${PROJECT_SOURCE_DIR}/include ${joltphysics_SOURCE_DIR}/Jolt)
target_link_libraries(e2e_verification PRIVATE Jolt)
# Match main app CRT so we don't mix heaps with Jolt (avoids STATUS_HEAP_CORRUPTION 0xC0000374)
if(MSVC)
  set_property(TARGET e2e_verification PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
  target_compile_options(e2e_verification PRIVATE
    $<$<CONFIG:Release>:/MD>
    $<$<CONFIG:Debug>:/MDd>)
  target_link_options(e2e_verification PRIVATE
    $<$<CONFIG:Release>:/NODEFAULTLIB:LIBCMT>
    $<$<CONFIG:Debug>:/NODEFAULTLIB:LIBCMTd>)
endif()
add_test(NAME E2EVerification COMMAND e2e_verification WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

# Headless exe: run real app; script verifies stdout (root Y >= 0.3) so rendered sim is stable
add_test(NAME HeadlessExe
  COMMAND ${CMAKE_COMMAND}
    -D "EXE=$<TARGET_FILE:biomechanics_simulator>"
    -D "SCRIPT_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
    -D "WORKING_DIR=${PROJECT_SOURCE_DIR}"
    -P ${CMAKE_CURRENT_SOURCE_DIR}/run_headless_verify.cmake
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
