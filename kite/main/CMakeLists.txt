idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")

# Build static library, do not build test executables
option(BUILD_SHARED_LIBS OFF)
option(BUILD_TESTING OFF)

# Import kitemath targets
add_subdirectory(lib/kitemath)

# Link kitemath to main component
target_link_libraries(${COMPONENT_LIB} PUBLIC kitemath)