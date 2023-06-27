#   Copyright (c) 2023 Christoph Hellmann Santos
#                      Błażej Sowa
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
macro(
    generate_dcf
    TARGET)
    add_custom_target(
        ${TARGET}_prepare ALL
        COMMAND rm -rf ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/${TARGET}/*
        COMMAND rm -rf ${CMAKE_BINARY_DIR}/config/${TARGET}/*
        COMMAND mkdir -p ${CMAKE_BINARY_DIR}/config/${TARGET}
        #COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/${TARGET}/
    )

    add_custom_target(
        ${TARGET} ALL
        DEPENDS ${TARGET}_prepare
    )

    add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND sed 's|@BUS_CONFIG_PATH@|${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/${TARGET}|g'
            ${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}/bus.yml > ${CMAKE_BINARY_DIR}/config/${TARGET}/bus.yml
        COMMAND dcfgen -v -d ${CMAKE_BINARY_DIR}/config/${TARGET}/ -rS ${CMAKE_BINARY_DIR}/config/${TARGET}/bus.yml
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}/
    )

    install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}/
        DESTINATION share/${PROJECT_NAME}/config/${TARGET}/
        PATTERN "bus.yml" EXCLUDE
    )

    install(DIRECTORY
        ${CMAKE_BINARY_DIR}/config/${TARGET}/
        DESTINATION share/${PROJECT_NAME}/config/${TARGET}/
    )

endmacro()

macro(
    cogen_dcf
    TARGET)
    add_custom_target(
        ${TARGET}_prepare ALL
        COMMAND rm -rf ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/${TARGET}/*
        COMMAND rm -rf ${CMAKE_BINARY_DIR}/config/${TARGET}/*
        COMMAND mkdir -p ${CMAKE_BINARY_DIR}/config/${TARGET}
        #COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/${TARGET}/
    )

    add_custom_target(
        ${TARGET} ALL
        DEPENDS ${TARGET}_prepare
    )

    add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND cogen --input-file ${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}/bus.yml --output-file ${CMAKE_BINARY_DIR}/config/${TARGET}/preprocessed_bus.yml
        COMMAND sed 's|@BUS_CONFIG_PATH@|${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/${TARGET}|g'
            ${CMAKE_BINARY_DIR}/config/${TARGET}/preprocessed_bus.yml > ${CMAKE_BINARY_DIR}/config/${TARGET}/bus.yml
        COMMAND dcfgen -v -d ${CMAKE_BINARY_DIR}/config/${TARGET}/ -rS ${CMAKE_BINARY_DIR}/config/${TARGET}/bus.yml
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}/
    )

    install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}/
        DESTINATION share/${PROJECT_NAME}/config/${TARGET}/
        PATTERN "bus.yml" EXCLUDE
    )

    install(DIRECTORY
        ${CMAKE_BINARY_DIR}/config/${TARGET}/
        DESTINATION share/${PROJECT_NAME}/config/${TARGET}/
    )

endmacro()

macro(dcfgen INPUT_DIR FILE OUTPUT_DIR)
    message(DEPRECATION "dcfgen macro is depreciated and will be remove in beta version. Use generate_dcf instead.")
    make_directory(${OUTPUT_DIR})
    add_custom_target(
        ${FILE} ALL
        COMMAND "dcfgen" "-d" ${OUTPUT_DIR} "-rS" ${INPUT_DIR}${FILE}
        WORKING_DIRECTORY ${INPUT_DIR}
        )
endmacro()
