# Olivier Stasse, Francois Keith, JRL, CNRS/AIST
# Creation: 4/11/2008
# History:
#
# Copyright CNRS/AIST

# For the test, we include the same files as the src
SET(${PROJECT_NAME}_test_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/include/ ${${PROJECT_NAME}_CXXFLAGS}")

# For the test, we link with the library corresponding to the src,  which is not installed yet
IF(UNIX)
  SET(${PROJECT_NAME}_test_CXXFLAGS "${${PROJECT_NAME}_test_CXXFLAGS} -DUNIX")
  SET(${PROJECT_NAME}_test_LDFLAGS "${LIBDIR_KW}${${PROJECT_NAME}_BINARY_DIR}/src ${LIBDIR_RP}${${PROJECT_NAME}_BINARY_DIR}/src ${LIBINCLUSION_KW}${PROJECT_NAME} ${${PROJECT_NAME}_src_LDFLAGS}")
ENDIF(UNIX)

IF(WIN32)
  SET(${PROJECT_NAME}_test_LDFLAGS "${LIBDIR_KW}${${PROJECT_NAME}_BINARY_DIR}/src  ${LIBINCLUSION_KW}${PROJECT_NAME}${LIB_EXT} ${${PROJECT_NAME}_src_LDFLAGS}")
ENDIF(WIN32)

MESSAGE(STATUS " unitTesting: ${UNITTESTING_CXXFLAGS}")
ADD_EXECUTABLE(	TestHumanoidDynamicRobot TestHumanoidDynamicRobot.cpp)
SET_TARGET_PROPERTIES(TestHumanoidDynamicRobot
	              PROPERTIES
		      COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
)

ADD_DEPENDENCIES (TestHumanoidDynamicRobot ${PROJECT_NAME})

ADD_EXECUTABLE(	TestSimpleLegRobot TestSimpleLegRobot.cpp)
SET_TARGET_PROPERTIES(TestSimpleLegRobot
	              PROPERTIES
		      COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
)
ADD_DEPENDENCIES (TestSimpleLegRobot ${PROJECT_NAME})

IF(GLUT_FOUND)
	ADD_EXECUTABLE(	TestHumanoidDynamicRobot2 TestHumanoidDynamicRobot2.cpp)
	SET_TARGET_PROPERTIES( TestHumanoidDynamicRobot2
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
	)
	ADD_DEPENDENCIES (TestHumanoidDynamicRobot2 ${PROJECT_NAME})
ENDIF(GLUT_FOUND)

ADD_EXECUTABLE(	TestBuildFromJointTree TestBuildFromJointTree.cpp)
SET_TARGET_PROPERTIES( TestBuildFromJointTree
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
          	      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}

)
ADD_DEPENDENCIES (TestBuildFromJointTree ${PROJECT_NAME})

ADD_EXECUTABLE(	TestCopyJointTreeThroughAbstractInterface TestCopyJointTreeThroughAbstractInterface.cpp)
SET_TARGET_PROPERTIES( TestCopyJointTreeThroughAbstractInterface
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
                      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
)
ADD_DEPENDENCIES (TestCopyJointTreeThroughAbstractInterface ${PROJECT_NAME})

ADD_EXECUTABLE(	Test2Models Test2Models.cpp)
SET_TARGET_PROPERTIES(Test2Models
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
)
ADD_DEPENDENCIES (Test2Models ${PROJECT_NAME})

#ADD_EXECUTABLE(	ComputeRealZMP ComputeRealZMP.cpp)
#SET_TARGET_PROPERTIES(ComputeRealZMP
#	              PROPERTIES
#	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
#		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
#)
#ADD_DEPENDENCIES (ComputeRealZMP ${PROJECT_NAME})
