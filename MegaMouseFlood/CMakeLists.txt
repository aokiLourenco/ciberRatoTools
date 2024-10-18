
set(robsample_SRC
    #Source
    mainMouse.cpp
    robfunc.cpp
)

add_executable(MegaMouse2 ${robsample_SRC})

target_include_directories(MegaMouse2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(MegaMouse2 RobSock)

set_target_properties(MegaMouse2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
