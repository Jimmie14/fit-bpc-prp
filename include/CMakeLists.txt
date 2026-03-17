add_library(manhattan_libs INTERFACE)

target_include_directories(manhattan_libs INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/
        ${CMAKE_CURRENT_SOURCE_DIR}/Controllers
        ${CMAKE_CURRENT_SOURCE_DIR}/Controllers/Node
)