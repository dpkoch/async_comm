get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(${SELF_DIR}/async_comm-targets.cmake)
get_filename_component(aysnc_comm_INCLUDE_DIRS "${SELF_DIR}/../../include/async_comm" ABSOLUTE)
set(async_comm_LIBRARIES async_comm)
