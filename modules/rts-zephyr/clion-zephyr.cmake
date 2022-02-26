if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
# Point to the build type and common config files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/prj.${CMAKE_BUILD_TYPE}.conf")
    list(APPEND CONF_FILE "prj.${CMAKE_BUILD_TYPE}.conf")
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD}.conf")
    list(APPEND CONF_FILE "boards/${BOARD}.conf")
endif()
list(APPEND CONF_FILE "prj.conf")
# CMake sets some default compiler flags in CMAKE_<LANG>_FLAGS_<CONFIG>.
# Zephyr sets all required flags from KConfig so
# clear CMake defaults to avoid any conflicts.
foreach(lang C CXX)
    set(CMAKE_${lang}_FLAGS "")
    foreach(type DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
        set(CMAKE_${lang}_FLAGS_${type} "")
    endforeach()
endforeach()
# Suppress a warning from Zephyr about a mismatch between
# CMAKE_<LANG>_FLAGS_<CONFIG> optimization flags (now empty) and
# Zephyr's KConfig optimization flags.
set(NO_BUILD_TYPE_WARNING ON)
list(APPEND ZEPHYR_EXTRA_MODULES
        ${CMAKE_CURRENT_SOURCE_DIR}/../modules/rts-zephyr)
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../modules/rts-zephyr)
find_package(Zephyr 3.0.0 EXACT)
