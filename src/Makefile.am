# 
#  Copyright 
# 

lib_LTLIBRARIES = libdynamicsJRLJapan.la
libdynamicsJRLJapan_la_SOURCES = \
	fileReader.cpp \
	Joint.cpp \
	Body.cpp \
	DynamicBody.cpp \
	MultiBody.cpp \
	DynamicMultiBody.cpp \
	HumanoidDynamicMultiBody.cpp \
	HumanoidSpecificities.cpp 

libdynamicsJRLJapan_la_LDFLAGS = \
	@ROBOTDYNAMICS_LIBS@

libdynamicsJRLJapan_la_CPPFLAGS = \
	-I$(srcdir)/../include/robotDynamicsJRLJapan	\
	@ROBOTDYNAMICS_CFLAGS@
