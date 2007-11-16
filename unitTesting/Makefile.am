# 
#  Copyright 
# 


noinst_PROGRAMS = TestHumanoidDynamicRobot TestHumanoidDynamicRobotglut TestBuildFromJointTree TestBuildFromJointTreeMathieu

TestHumanoidDynamicRobot_SOURCES = TestHumanoidDynamicRobot.cpp 

TestHumanoidDynamicRobot_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@

TestHumanoidDynamicRobot_LDADD = -L../src -ldynamicsJRLJapan @ROBOTDYNAMICS_LIBS@

TestHumanoidDynamicRobotglut_SOURCES = TestHumanoidDynamicRobotglut.cpp 

TestHumanoidDynamicRobotglut_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@

TestHumanoidDynamicRobotglut_LDADD = -L../src -ldynamicsJRLJapan @ROBOTDYNAMICS_LIBS@ -lglut -lGL

TestBuildFromJointTree_SOURCES = TestBuildFromJointTree.cpp 

TestBuildFromJointTree_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@

TestBuildFromJointTree_LDADD = -L../src -ldynamicsJRLJapan @ROBOTDYNAMICS_LIBS@ -lglut -lGL

TestBuildFromJointTreeMathieu_SOURCES = TestBuildFromJointTreeMathieu.cpp 

TestBuildFromJointTreeMathieu_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@

TestBuildFromJointTreeMathieu_LDADD = -L../src -ldynamicsJRLJapan @ROBOTDYNAMICS_LIBS@ -lglut -lGL

#TestPb1_SOURCES = TestPb1.cpp 

#TestPb1_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@

#TestPb1_LDADD = -L../src -ldynamicsJRLJapan @ROBOTDYNAMICS_LIBS@ 
