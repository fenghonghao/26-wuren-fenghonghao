aux_source_directory(src A1_SRC_LIST)

add_library(A1 ${A1_SRC_LIST})

target_include_directories(A1 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)