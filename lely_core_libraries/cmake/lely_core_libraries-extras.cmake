# dcfgen macro executes the dcfgen command for the specified target.
# For this macro to work there needs to be a folder config/{target}
# in your package. Inside that folder there needs to be the bus.yml
# file to use for generation.
macro(
    dcfgen 
    TARGET)

    add_custom_target(
        ${TARGET} ALL    
    )
    
    add_custom_command(
        TARGET ${TARGET} PRE_BUILD
        COMMAND mkdir -p ${CMAKE_BINARY_DIR}/config/${TARGET}/
    )

    add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND dcfgen -d ${CMAKE_BINARY_DIR}/config/${TARGET}/ -rS bus.yml
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}/
    )

    install(DIRECTORY
        config/${TARGET}/
        DESTINATION share/${PROJECT_NAME}/config/${TARGET}/
    )
  
    install(DIRECTORY
        ${CMAKE_BINARY_DIR}/config/${TARGET}/
        DESTINATION share/${PROJECT_NAME}/config/${TARGET}/
    )

endmacro()