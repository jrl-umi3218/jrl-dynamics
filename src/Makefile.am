# 
#  Copyright 
# 

lib_LTLIBRARIES = libdynamicsJRLJapan.la
libdynamicsJRLJapan_la_SOURCES = \
	fileReader.cpp \
	Joint.cpp \
	Body.cpp \
	Hand.cpp \
	DynamicBody.cpp \
	MultiBody.cpp \
	DynamicMultiBody.cpp \
	HumanoidDynamicMultiBody.cpp \
	HumanoidSpecificities.cpp  \
	SpiritVRMLReader.cpp

libdynamicsJRLJapan_la_LDFLAGS = \
	@ROBOTDYNAMICS_LIBS@	\
	@MAL_LIBS@ \
	-release ${PACKAGE_VERSION}

libdynamicsJRLJapan_la_CPPFLAGS = \
	-I$(srcdir)/../include	\
	@ROBOTDYNAMICS_CFLAGS@ \
	@MAL_CFLAGS@
