if(CONFIG_OWNTECH_SAFETY_API OR CONFIG_OWNTECH_SAFETY_SUPERVISION)
  # Select directory to add to the include path
  zephyr_include_directories (./public_api)

  # Define the current folder as a Zephyr library
  zephyr_library()

  # CPU supervision: always on whenever the HRTIM driver is built (ADR 0009)
  if(CONFIG_OWNTECH_SAFETY_SUPERVISION)
    zephyr_library_sources(src/safety_supervision.c src/safety_trips.c)
    # Disable the PWM at the first C instruction of the fault path, before
    # Zephyr's fault logging (see safety_supervision.c). A Zephyr update that
    # renames these functions makes the link fail instead of silently losing it.
    zephyr_ld_options(-Wl,--wrap=z_arm_fault -Wl,--wrap=z_fatal_error)

    # Protection allocation (ADR 0009, #17): from the parsed devicetree, assign
    # each limit its hardware path; write the protection manifest, the board's
    # protection table and a generated header. A hardware limit with no path and
    # no exception stops the build here.
    set(SAFETY_OUT_DIR ${PROJECT_BINARY_DIR}/safety)
    set(SAFETY_PLAN_HEADER ${PROJECT_BINARY_DIR}/include/generated/safety_protection_plan.h)
    set(SAFETY_REPORT_ONLY_ARG "")
    if(DEFINED SAFETY_CHECK_REPORT_ONLY)
      set(SAFETY_REPORT_ONLY_ARG --report-only)
    endif()
    execute_process(
      COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/../scripts/protection_manifest.py
              --edt ${EDT_PICKLE} --zephyr-base ${ZEPHYR_BASE}
              --out-dir ${SAFETY_OUT_DIR} --header ${SAFETY_PLAN_HEADER} ${SAFETY_REPORT_ONLY_ARG}
      OUTPUT_VARIABLE SAFETY_ALLOC_OUT
      ERROR_VARIABLE SAFETY_ALLOC_ERR
      RESULT_VARIABLE SAFETY_ALLOC_RC)
    string(REPLACE ";" "#SEMI#" SAFETY_ALLOC_OUT "${SAFETY_ALLOC_OUT}")
    string(REPLACE "\n" ";" SAFETY_ALLOC_LINES "${SAFETY_ALLOC_OUT}")
    foreach(SAFETY_LINE ${SAFETY_ALLOC_LINES})
      string(REPLACE "#SEMI#" ";" SAFETY_LINE "${SAFETY_LINE}")
      if(SAFETY_LINE MATCHES "^WARNING: ")
        string(REGEX REPLACE "^WARNING: " "" SAFETY_LINE "${SAFETY_LINE}")
        message(WARNING "safety: ${SAFETY_LINE}")
      elseif(NOT SAFETY_LINE STREQUAL "")
        message(STATUS "${SAFETY_LINE}")
      endif()
    endforeach()
    if(NOT SAFETY_ALLOC_RC EQUAL 0)
      message(FATAL_ERROR "safety: the protection allocation failed (ADR 0009). "
                          "Each ERROR line above names a limit and how to fix it.\n${SAFETY_ALLOC_ERR}")
    endif()
  endif()

  # Shield-level safety API
  if(CONFIG_OWNTECH_SAFETY_API)
    zephyr_library_sources(
      src/safety_setting.cpp
      src/safety_shield.cpp
      public_api/SafetyAPI.cpp
      )
  endif()
endif()
