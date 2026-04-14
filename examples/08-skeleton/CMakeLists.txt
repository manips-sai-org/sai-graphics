set(EXAMPLE_NAME 08-skeleton)

# create an executable
ADD_EXECUTABLE (${EXAMPLE_NAME} main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (${EXAMPLE_NAME}
	${SAI-GRAPHICS_EXAMPLES_LIBRARIES}
)

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY graphics muscles.xml models/rajagopal_model/Rajagopal2015.urdf models/rajagopal_model/Geometry DESTINATION ${RESOURCE_DIR})
