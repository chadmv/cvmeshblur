set(SOURCE_FILES
    "pluginMain.cpp"
    "cvMeshBlurCmd.cpp"
    "cvMeshBlurCmd.h"
    "cvMeshBlurDeformer.cpp"
    "cvMeshBlurDeformer.h"
)

find_package(Maya REQUIRED)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE Maya::Maya)
target_include_directories(${PROJECT_NAME} PRIVATE Maya::Maya)
MAYA_PLUGIN(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE} DESTINATION plug-ins/${MAYA_VERSION})

