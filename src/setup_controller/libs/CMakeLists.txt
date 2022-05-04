add_subdirectory(animation)

add_library(third_party INTERFACE)

target_link_libraries(third_party INTERFACE animation)
target_include_directories(third_party INTERFACE animation)




