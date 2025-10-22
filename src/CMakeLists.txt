CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT(FmPDR)

MESSAGE(STATUS "###Start building ${PROJECT_NAME}###")

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "-Wno-literal-suffix")

AUX_SOURCE_DIRECTORY(. DIR_SRCS)

ADD_DEFINITIONS(-D_LINUX)

IF("${CMAKE_BUILD_TYPE}" STREQUAL "debug" OR "${CMAKE_BUILD_TYPE}" STREQUAL "")
    ADD_COMPILE_OPTIONS(-Wall -gdwarf-2 -fstack-protector-all -g)
ELSE()
    ADD_COMPILE_OPTIONS(-O2 -Wall -fstack-protector-all)
ENDIF()

INCLUDE_DIRECTORIES(${PROJECT_NAME}
    PRIVATE
    ${CMAKE_INSTALL_PREFIX}/include
    ${CMAKE_INSTALL_PREFIX}/include/Fusion
    ${CMAKE_INSTALL_PREFIX}/include/eigen3
    )

LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib/)

ADD_LIBRARY(${PROJECT_NAME} STATIC ${DIR_SRCS})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC -Wl,-z,relro,-z,now)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC iir_static dlib openblas Fusion GeographicLib)

SET(RUNTIME_DEST bin)
SET(INCLUDE_DEST include/FmPDR)
SET(LIBRARY_DEST lib)
SET(CONFIG_DEST conf)

INSTALL (TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${RUNTIME_DEST}
    LIBRARY DESTINATION ${LIBRARY_DEST}
    ARCHIVE DESTINATION ${LIBRARY_DEST}
    )

INSTALL(FILES fm_pdr.h
    data_buffer_loader.h
    data_file_loader.h
    data_manager.h
    json_operator.h
    pdr.h
    merge_direction_step.h
    direction_predictor.h
    step_predictor.h
    DESTINATION ${INCLUDE_DEST})
INSTALL(FILES conf/config.json DESTINATION ${CONFIG_DEST})