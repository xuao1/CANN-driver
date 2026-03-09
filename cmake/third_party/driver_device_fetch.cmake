# ------------------------------------------------------------------------------------------------------------
# Copyright (c) 2025 Huawei Technologies Co., Ltd.
# This program is free software, you can redistribute it and/or modify it under the terms and conditions of
# CANN Open Software License Agreement Version 2.0 (the "License").
# Please refer to the License for details. You may not use this file except in compliance with the License.
# THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND, EITHER EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT, MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
# See LICENSE in the root of the software repository for the full text of the License.
# ------------------------------------------------------------------------------------------------------------

set(DRIVER_DEVICE_NAME "driver-device")
set(DRIVER_DEVICE_SERVER "https://ascend.devcloud.huaweicloud.com/artifactory/cann-run-mirror/dependency")
set(DRIVER_DEVICE_VERSION "8.5.0-beta.1")
set(DRIVER_DEVICE_DATE "20260127000324761")

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

set(DRIVER_DEVICE_INSTALL_DIR ${EXTERN_DEPEND_SOURCE_DIR}/driver_device)
file(MAKE_DIRECTORY "${DRIVER_DEVICE_INSTALL_DIR}")

file(GLOB DRIVER_DEVICE_ARCHIVE_FILE ${CMAKE_BINARY_DIR}/*driver-device*.tar.gz)
if(NOT DRIVER_DEVICE_ARCHIVE_FILE)
    if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")
        set(DRIVER_DEVICE_PLATFORM "x86_64")
    elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "aarch64")
        set(DRIVER_DEVICE_PLATFORM "aarch64")
    else()
        message(FATAL_ERROR "Unknown platform: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
    endif()

    if(${PRODUCT} STREQUAL ascend910B)
        if(ASCEND910_93_EX)
            set(DRIVER_DEVICE_PRODUCT "A3")
        else()
            set(DRIVER_DEVICE_PRODUCT "910b")
        endif()
    else()
        message(FATAL_ERROR "Unknown COMPUTE_UNIT: ${PRODUCT}")
    endif()

    if(ENABLE_BUILD_PRODUCT)
        set(DRIVER_DEVICE_RELEASE "product")
    else()
        set(DRIVER_DEVICE_RELEASE "demo")
    endif()

    set(DRIVER_DEVICE_FILE "cann-driver-device-${DRIVER_DEVICE_PRODUCT}-${DRIVER_DEVICE_RELEASE}_${DRIVER_DEVICE_VERSION}_linux-${CMAKE_HOST_SYSTEM_PROCESSOR}.tar.gz")
    set(DRIVER_DEVICE_URL "${DRIVER_DEVICE_SERVER}/${DRIVER_DEVICE_VERSION}/${DRIVER_DEVICE_DATE}/${DRIVER_DEVICE_PLATFORM}/basic/${DRIVER_DEVICE_FILE}")
    message(STATUS "Not find driver-device package, downloading ${DRIVER_DEVICE_NAME} from ${DRIVER_DEVICE_URL}")
    include(FetchContent)
    FetchContent_Declare(
        ${DRIVER_DEVICE_NAME}
        URL ${DRIVER_DEVICE_URL}
        DOWNLOAD_DIR ${EXTERN_DEPEND_DOWNLOAD_DIR}
        SOURCE_DIR ${DRIVER_DEVICE_INSTALL_DIR}
    )
    FetchContent_MakeAvailable(${DRIVER_DEVICE_NAME})
    FetchContent_GetProperties(${DRIVER_DEVICE_NAME})
    if(NOT ${DRIVER_DEVICE_NAME}_POPULATED)
        message(FATAL_ERROR "Failed to download ${DRIVER_DEVICE_NAME} from ${DRIVER_DEVICE_URL}")
    endif()

    set(DRIVER_DEVICE_ARCHIVE_FILE ${EXTERN_DEPEND_DOWNLOAD_DIR}/${DRIVER_DEVICE_FILE})
    add_custom_target(extract_driver_device_pre ALL
        COMMAND echo "Extracting driver-device package to ${DRIVER_DEVICE_INSTALL_DIR}."
        DEPENDS ${DRIVER_DEVICE_ARCHIVE_FILE}
        WORKING_DIRECTORY ${DRIVER_DEVICE_INSTALL_DIR}
    )
    set(DRV_DEV_ARCHIVE_PREFIX ${DRIVER_DEVICE_INSTALL_DIR}/driver)
else()
    message(STATUS "Find driver-device package: ${DRIVER_DEVICE_ARCHIVE_FILE}.")
    get_filename_component(DRIVER_DEVICE_ARCHIVE_NAME "${DRIVER_DEVICE_ARCHIVE_FILE}" NAME)
    add_custom_target(extract_driver_device_pre ALL
        COMMAND echo "Extracting driver-device package to ${DRIVER_DEVICE_INSTALL_DIR}."
        COMMAND ${CMAKE_COMMAND} -E copy ${DRIVER_DEVICE_ARCHIVE_FILE} ${DRIVER_DEVICE_INSTALL_DIR}
        COMMAND ${CMAKE_COMMAND} -E tar -xf ${DRIVER_DEVICE_INSTALL_DIR}/${DRIVER_DEVICE_ARCHIVE_NAME}
        DEPENDS ${DRIVER_DEVICE_ARCHIVE_FILE}
        WORKING_DIRECTORY ${DRIVER_DEVICE_INSTALL_DIR}
    )
    set(DRV_DEV_ARCHIVE_PREFIX ${DRIVER_DEVICE_INSTALL_DIR}/driver/driver)
endif()

function(get_driver_device)
    add_custom_target(extract_driver_device ALL
        COMMAND echo "copying driver-device file to ${CMAKE_BINARY_DIR}."
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/lib/host
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/lib/tools
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/lib/lib64/common
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/lib/script
        COMMAND ${CMAKE_COMMAND} -E copy ${DRV_DEV_ARCHIVE_PREFIX}/device/* ${CMAKE_BINARY_DIR}/lib/host/
        COMMAND ${CMAKE_COMMAND} -E copy ${DRV_DEV_ARCHIVE_PREFIX}/tools/ascend_check.bin ${CMAKE_BINARY_DIR}/lib/tools/
        COMMAND ${CMAKE_COMMAND} -E copy ${DRV_DEV_ARCHIVE_PREFIX}/tools/upgrade-tool ${CMAKE_BINARY_DIR}/lib/tools/
        COMMAND ${CMAKE_COMMAND} -E copy ${DRV_DEV_ARCHIVE_PREFIX}/tools/hccn_tool ${CMAKE_BINARY_DIR}/lib/tools/
        COMMAND ${CMAKE_COMMAND} -E copy ${DRV_DEV_ARCHIVE_PREFIX}/lib64/common/dcache_lock_mix.o ${CMAKE_BINARY_DIR}/lib/lib64/common/
        COMMAND ${CMAKE_COMMAND} -E copy ${DRV_DEV_ARCHIVE_PREFIX}/lib64/common/libtls_adp.so ${CMAKE_BINARY_DIR}/lib/lib64/common/
        COMMAND ${CMAKE_COMMAND} -E copy ${DRV_DEV_ARCHIVE_PREFIX}/lib64/common/libascend_kms.so ${CMAKE_BINARY_DIR}/lib/lib64/common/
        COMMAND ${CMAKE_COMMAND} -E copy ${DRV_DEV_ARCHIVE_PREFIX}/script/hccn_weak_dict.conf ${CMAKE_BINARY_DIR}/lib/script/
        DEPENDS extract_driver_device_pre
        WORKING_DIRECTORY ${DRIVER_DEVICE_INSTALL_DIR}
    )
    if(ENABLE_BUILD_PRODUCT)
        include(${CMAKE_SOURCE_DIR}/src/custom/cmake/driver_device_fetch.cmake)
    endif()
endfunction()
