INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src ${GNULIBS_INCLUDE_DIR})
LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES} )

find_package (Threads)

ADD_EXECUTABLE(aruco_test aruco_test.cpp)
ADD_EXECUTABLE(aruco_simple aruco_simple.cpp)
ADD_EXECUTABLE(aruco_create_marker aruco_create_marker.cpp)
ADD_EXECUTABLE(aruco_create_board aruco_create_board.cpp)
ADD_EXECUTABLE(aruco_selectoptimalmarkers aruco_selectoptimalmarkers.cpp)
ADD_EXECUTABLE(aruco_simple_board aruco_simple_board.cpp)
ADD_EXECUTABLE(aruco_test_board aruco_test_board.cpp)
ADD_EXECUTABLE(aruco_board_pix2meters aruco_board_pix2meters.cpp)
ADD_EXECUTABLE(aruco_calibration aruco_calibration.cpp)
ADD_EXECUTABLE(marker_distance_test marker_distance_test.cpp)
ADD_EXECUTABLE(video_recorder video_recorder.cpp)
ADD_EXECUTABLE(distance_calibration distance_calibration.cpp)
#ADD_EXECUTABLE(aruco_test_board_stability aruco_test_board_stability.cpp)

#TARGET_LINK_LIBRARIES ( test ${CMAKE_THREAD_LIBS_INIT} )
target_link_libraries (distance_calibration ${CMAKE_THREAD_LIBS_INIT})

INSTALL(TARGETS aruco_test  aruco_board_pix2meters aruco_simple aruco_create_marker aruco_create_board aruco_simple_board aruco_test_board aruco_selectoptimalmarkers RUNTIME DESTINATION bin)
IF(GL_FOUND)
  ADD_EXECUTABLE(aruco_test_gl aruco_test_gl.cpp)
  TARGET_LINK_LIBRARIES(aruco_test_gl ${OPENGL_LIBS})

  ADD_EXECUTABLE(aruco_test_board_gl aruco_test_board_gl.cpp)
  TARGET_LINK_LIBRARIES(aruco_test_board_gl ${OPENGL_LIBS})

  INSTALL(TARGETS aruco_test_gl aruco_test_board_gl RUNTIME DESTINATION bin)
ENDIF(GL_FOUND)
