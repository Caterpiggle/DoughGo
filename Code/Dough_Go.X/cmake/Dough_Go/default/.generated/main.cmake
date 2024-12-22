include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")
set(Dough_Go_default_library_list )
# Handle files with suffix (s|as|asm|AS|ASM|As|aS|Asm) 
if(Dough_Go_default_FILE_GROUP_assemble)
    add_library(Dough_Go_default_assemble OBJECT ${Dough_Go_default_FILE_GROUP_assemble})
    Dough_Go_default_assemble_rule(Dough_Go_default_assemble)
    list(APPEND Dough_Go_default_library_list "$<TARGET_OBJECTS:Dough_Go_default_assemble>")
endif()

# Handle files with suffix S 
if(Dough_Go_default_FILE_GROUP_assemblePreprocess)
    add_library(Dough_Go_default_assemblePreprocess OBJECT ${Dough_Go_default_FILE_GROUP_assemblePreprocess})
    Dough_Go_default_assemblePreprocess_rule(Dough_Go_default_assemblePreprocess)
    list(APPEND Dough_Go_default_library_list "$<TARGET_OBJECTS:Dough_Go_default_assemblePreprocess>")
endif()

# Handle files with suffix [cC] 
if(Dough_Go_default_FILE_GROUP_compile)
    add_library(Dough_Go_default_compile OBJECT ${Dough_Go_default_FILE_GROUP_compile})
    Dough_Go_default_compile_rule(Dough_Go_default_compile)
    list(APPEND Dough_Go_default_library_list "$<TARGET_OBJECTS:Dough_Go_default_compile>")
endif()

if (BUILD_LIBRARY)
        message(STATUS "Building LIBRARY")
        add_library(${Dough_Go_default_image_name} ${Dough_Go_default_library_list})
        foreach(lib ${Dough_Go_default_FILE_GROUP_link})
        target_link_libraries(${Dough_Go_default_image_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR} /${lib})
        endforeach()
        add_custom_command(
            TARGET ${Dough_Go_default_image_name}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${Dough_Go_default_output_dir}
    COMMAND ${CMAKE_COMMAND} -E copy lib${Dough_Go_default_image_name}.a ${Dough_Go_default_output_dir}/${Dough_Go_default_original_image_name})
else()
    message(STATUS "Building STANDARD")
    add_executable(${Dough_Go_default_image_name} ${Dough_Go_default_library_list})
    foreach(lib ${Dough_Go_default_FILE_GROUP_link})
    target_link_libraries(${Dough_Go_default_image_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/${lib})
endforeach()
    Dough_Go_default_link_rule(${Dough_Go_default_image_name})
    
add_custom_command(
    TARGET ${Dough_Go_default_image_name}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${Dough_Go_default_output_dir}
    COMMAND ${CMAKE_COMMAND} -E copy ${Dough_Go_default_image_name} ${Dough_Go_default_output_dir}/${Dough_Go_default_original_image_name})
endif()
