# 
#  Copyright 
# 


noinst_PROGRAMS = TestHumanoidDynamicRobot TestPb1

TestHumanoidDynamicRobot_SOURCES = TestHumanoidDynamicRobot.cpp 

TestHumanoidDynamicRobot_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@

TestHumanoidDynamicRobot_LDADD = -L../src -ldynamicsJRLJapan @ROBOTDYNAMICS_LIBS@

TestPb1_SOURCES = TestPb1.cpp 

TestPb1_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@

TestPb1_LDADD = -L../src -ldynamicsJRLJapan @ROBOTDYNAMICS_LIBS@
