add_subdirectory(SoftNeckControl)
add_subdirectory(SoftArmControl)
add_subdirectory(SoftNeckImu)
add_subdirectory(SoftNeckMocap)

set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES} ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
set(SUBDIR_LINK_NAMES ${SUBDIR_LINK_NAMES} ${PROJECT_NAME} PARENT_SCOPE)

