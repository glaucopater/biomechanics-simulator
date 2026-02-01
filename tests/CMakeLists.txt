# Smoke test: link against Jolt and run a minimal check
add_executable(smoke_test smoke_test.cpp)
target_include_directories(smoke_test PRIVATE ${PROJECT_SOURCE_DIR}/include ${joltphysics_SOURCE_DIR}/Jolt)
target_link_libraries(smoke_test PRIVATE Jolt)
add_test(NAME SmokeTest COMMAND smoke_test)
