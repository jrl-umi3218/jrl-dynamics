# 
#  Copyright 
# 


noinst_PROGRAMS = TestHumanoidDynamicRobot TestHumanoidDynamicRobotglut TestBuildFromJointTree TestBuildFromJointTreeMathieu

TestHumanoidDynamicRobot_SOURCES = TestHumanoidDynamicRobot.cpp 

TestHumanoidDynamicRobot_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

TestHumanoidDynamicRobot_LDADD = -L../src -ldynamicsJRLJapan @ROBOTDYNAMICS_LIBS@ @MAL_LIBS@


TestHumanoidDynamicRobotglut_SOURCES = TestHumanoidDynamicRobotglut.cpp 

TestHumanoidDynamicRobotglut_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

TestHumanoidDynamicRobotglut_LDADD = ../src/libdynamicsJRLJapan.la @ROBOTDYNAMICS_LIBS@ @MAL_LIBS@ -lglut -lGL

TestBuildFromJointTree_SOURCES = TestBuildFromJointTree.cpp 

TestBuildFromJointTree_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

TestBuildFromJointTree_LDADD = ../src/libdynamicsJRLJapan.la @ROBOTDYNAMICS_LIBS@ @MAL_LIBS@ -lglut -lGL

TestBuildFromJointTreeMathieu_SOURCES = TestBuildFromJointTreeMathieu.cpp 

TestBuildFromJointTreeMathieu_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

TestBuildFromJointTreeMathieu_LDADD = ../src/libdynamicsJRLJapan.la @ROBOTDYNAMICS_LIBS@ @MAL_LIBS@ -lglut -lGL

#TestPb1_SOURCES = TestPb1.cpp 

#TestPb1_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

#TestPb1_LDADD = -L../src -ldynamicsJRLJapan @ROBOTDYNAMICS_LIBS@ @MAL_LIBS@ 
