aux_source_directory(src A2_SRC_LIST)

add_library(A2 ${A2_SRC_LIST})

target_include_directories(A2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)