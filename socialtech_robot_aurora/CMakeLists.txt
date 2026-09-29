cmake_minimum_required(VERSION 3.8)
project(socialtech_robot_aurora)

find_package(ament_cmake REQUIRED)

set(AURORA_REMOTE_PUBLIC_PLATFORM "")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(AURORA_REMOTE_PUBLIC_PLATFORM "linux_x86_64")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    set(AURORA_REMOTE_PUBLIC_PLATFORM "linux_aarch64")
  endif()
endif()

if(AURORA_REMOTE_PUBLIC_PLATFORM)
  get_filename_component(
    AURORA_REMOTE_PUBLIC_LIBRARY_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/../../aurora_ros/src/aurora_remote_public/lib/${AURORA_REMOTE_PUBLIC_PLATFORM}"
    ABSOLUTE
  )

  if(EXISTS "${AURORA_REMOTE_PUBLIC_LIBRARY_PATH}/libslamtec_aurora_remote_sdk.so")
    ament_environment_hooks(
      "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/aurora_remote_public_library_path.dsv.in"
    )
  else()
    message(WARNING
      "Aurora remote public library not found at "
      "${AURORA_REMOTE_PUBLIC_LIBRARY_PATH}. "
      "LD_LIBRARY_PATH will not be configured automatically."
    )
  endif()
else()
  message(WARNING
    "Unsupported Aurora remote public platform for "
    "${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}. "
    "Set LD_LIBRARY_PATH manually if needed."
  )
endif()

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/aurora_base_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
