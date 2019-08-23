add_executable (grasp_driver main.cpp canAPI.cpp)
target_link_libraries (grasp_driver BHand pcanbasic pthread hiredis ${ALL_LIBS})
#install (TARGETS grasp DESTINATION ${PROJECT_SOURCE_DIR}/bin)
#install (TARGETS grasp_driver DESTINATION ${PROJECT_BINARY_DIR}/bin)

# export resources such as model files.
# NOTE: this requires an install build
FILE(COPY hand.urdf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
