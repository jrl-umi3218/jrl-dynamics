# Olivier Stasse, JRL, CNRS/AIST
# Creation: 4/11/2008
# History:
#
# Copyright CNRS/AIST

ADD_EXECUTABLE(	TestHumanoidDynamicRobot TestHumanoidDynamicRobot.cpp)
SET_TARGET_PROPERTIES(TestHumanoidDynamicRobot
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_LDFLAGS}
)

IF(GLUT_FOUND)
ADD_EXECUTABLE(	TestHumanoidDynamicRobot2 TestHumanoidDynamicRobot2.cpp)
SET_TARGET_PROPERTIES( TestHumanoidDynamicRobot2
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_LDFLAGS}
)

ENDIF(GLUT_FOUND)

ADD_EXECUTABLE(	TestBuildFromJointTree TestBuildFromJointTree.cpp)
SET_TARGET_PROPERTIES( TestBuildFromJointTree
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_LDFLAGS}
)

ADD_EXECUTABLE(	TestCopyJointTreeThroughAbstractInterface TestCopyJointTreeThroughAbstractInterface.cpp)
SET_TARGET_PROPERTIES( TestCopyJointTreeThroughAbstractInterface
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_LDFLAGS}
)

ADD_EXECUTABLE(	Test2Models Test2Models.cpp)
SET_TARGET_PROPERTIES(Test2Models
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_LDFLAGS}
)

