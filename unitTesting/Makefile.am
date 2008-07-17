# 
#  Copyright 
# 

noinst_PROGRAMS=\
			TestHumanoidDynamicRobot	\
			TestBuildFromJointTree		\
			TestCopyJointTreeThroughAbstractInterface \
			Test2Models

if HAVE_GLUT
noinst_PROGRAMS+=	TestHumanoidDynamicRobotglut
endif

TestHumanoidDynamicRobot_SOURCES = TestHumanoidDynamicRobot.cpp 

TestHumanoidDynamicRobot_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

TestHumanoidDynamicRobot_LDADD = -L../src -ldynamicsJRLJapan @ROBOTDYNAMICS_LIBS@ @MAL_LIBS@ 


TestHumanoidDynamicRobotglut_SOURCES = TestHumanoidDynamicRobotglut.cpp 

TestHumanoidDynamicRobotglut_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

TestHumanoidDynamicRobotglut_LDADD = ../src/libdynamicsJRLJapan.la @ROBOTDYNAMICS_LIBS@ @MAL_LIBS@ -lglut -lGL

TestBuildFromJointTree_SOURCES = TestBuildFromJointTree.cpp 

TestBuildFromJointTree_CPPFLAGS=\
			-I$(srcdir)/../include		\
			@ROBOTDYNAMICS_CFLAGS@		\
			@MAL_CFLAGS@

TestBuildFromJointTree_LDADD = ../src/libdynamicsJRLJapan.la @ROBOTDYNAMICS_LIBS@ @MAL_LIBS@

TestCopyJointTreeThroughAbstractInterface_SOURCES = TestCopyJointTreeThroughAbstractInterface.cpp 

TestCopyJointTreeThroughAbstractInterface_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

TestCopyJointTreeThroughAbstractInterface_LDADD=\
			../src/libdynamicsJRLJapan.la	\
			@ROBOTDYNAMICS_LIBS@		\
			@MAL_LIBS@


Test2Models_SOURCES = Test2Models.cpp 

Test2Models_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

Test2Models_LDADD=\
		../src/libdynamicsJRLJapan.la	\
		@ROBOTDYNAMICS_LIBS@		\
		@MAL_LIBS@

#TestPb1_SOURCES = TestPb1.cpp 

#TestPb1_CPPFLAGS = -I$(srcdir)/../include @ROBOTDYNAMICS_CFLAGS@ @MAL_CFLAGS@

#TestPb1_LDADD = -L../src -ldynamicsJRLJapan @ROBOTDYNAMICS_LIBS@ @MAL_LIBS@ 
