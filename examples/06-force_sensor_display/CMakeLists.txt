set(EXAMPLE_NAME 06-force_sensor_display)

# create an executable
ADD_EXECUTABLE (${EXAMPLE_NAME} main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (${EXAMPLE_NAME}
	${SAI-GRAPHICS_EXAMPLES_LIBRARIES}
)