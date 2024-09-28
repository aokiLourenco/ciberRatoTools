
set(robsample_SRC
    #Source
    mainRob.c
    robfunc.c
)

add_executable(MegaMouse ${robsample_SRC})

target_include_directories(MegaMouse PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(MegaMouse RobSock)

set_target_properties(MegaMouse PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
