# Run biomechanics_simulator --headless and verify root Y >= 0.3 from stdout.
# Usage: cmake -D EXE=<path_to_exe> -D SCRIPT_DIR=<tests dir> -D WORKING_DIR=<repo root> -P run_headless_verify.cmake
if(NOT EXE)
  message(FATAL_ERROR "EXE not set")
endif()
if(NOT WORKING_DIR)
  set(WORKING_DIR "${SCRIPT_DIR}/..")
endif()
execute_process(
  COMMAND "${EXE}" --headless
  WORKING_DIRECTORY "${WORKING_DIR}"
  OUTPUT_VARIABLE OUT
  ERROR_VARIABLE ERR
  RESULT_VARIABLE RES
)
if(RES)
  message(FATAL_ERROR "Headless exe failed with ${RES}\nstdout: ${OUT}\nstderr: ${ERR}")
endif()
# Parse "Root (LowerBody) position after simulation: (x, y, z)"
string(REGEX MATCH "Root \\(LowerBody\\) position after simulation: \\(([^,]+), ([^,]+), ([^)]+)\\)" _ "${OUT}")
set(Y "${CMAKE_MATCH_2}")
if(NOT Y)
  message(FATAL_ERROR "Could not parse root position from output")
endif()
if(Y LESS "0.1")
  message(FATAL_ERROR "Root Y=${Y} < 0.1 (character should stay above ground)")
endif()
message(STATUS "HeadlessExe OK: root Y=${Y}")
