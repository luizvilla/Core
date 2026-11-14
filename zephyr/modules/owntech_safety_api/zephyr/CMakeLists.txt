if(CONFIG_OWNTECH_SAFETY_API OR CONFIG_OWNTECH_SAFETY_SUPERVISION)
  # Select directory to add to the include path
  zephyr_include_directories (./public_api)

  # Define the current folder as a Zephyr library
  zephyr_library()

  # CPU supervision: always on whenever the HRTIM driver is built (ADR 0009)
  if(CONFIG_OWNTECH_SAFETY_SUPERVISION)
    zephyr_library_sources(src/safety_supervision.c)
    # Disable the PWM at the first C instruction of the fault path, before
    # Zephyr's fault logging (see safety_supervision.c). A Zephyr update that
    # renames these functions makes the link fail instead of silently losing it.
    zephyr_ld_options(-Wl,--wrap=z_arm_fault -Wl,--wrap=z_fatal_error)
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
