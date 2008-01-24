# 
#  Copyright 
# 


noinst_PROGRAMS = TestHumanoidDynamicRobot TestHumanoidDynamicRobotglut TestBuildFromJointTree TestCopyJointTreeThroughAbstractInterface

TestHumanoidDynamicRobot_SOURCES = TestHumanoidDynamicRobot.cpp 

TestHumanoidDynamicRobot_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

TestHumanoidDynamicRobot_LDADD = -L../src -ldynamicsJRLJapan @ROBOTDYNAMICS_LIBS@ @MAL_LIBS@


TestHumanoidDynamicRobotglut_SOURCES = TestHumanoidDynamicRobotglut.cpp 

TestHumanoidDynamicRobotglut_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

TestHumanoidDynamicRobotglut_LDADD = ../src/libdynamicsJRLJapan.la @ROBOTDYNAMICS_LIBS@ @MAL_LIBS@ -lglut -lGL

TestBuildFromJointTree_SOURCES = TestBuildFromJointTree.cpp 

TestBuildFromJointTree_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

TestBuildFromJointTree_LDADD = ../src/libdynamicsJRLJapan.la @ROBOTDYNAMICS_LIBS@ @MAL_LIBS@ -lglut -lGL

TestCopyJointTreeThroughAbstractInterface_SOURCES = TestCopyJointTreeThroughAbstractInterface.cpp 

TestCopyJointTreeThroughAbstractInterface_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

TestCopyJointTreeThroughAbstractInterface_LDADD = ../src/libdynamicsJRLJapan.la @ROBOTDYNAMICS_LIBS@ @MAL_LIBS@ -lglut -lGL

#TestPb1_SOURCES = TestPb1.cpp 

#TestPb1_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

#TestPb1_LDADD = -L../src -ldynamicsJRLJapan @ROBOTDYNAMICS_LIBS@ @MAL_LIBS@ 
