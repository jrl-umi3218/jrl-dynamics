# 
#  Copyright 
# 

SUBDIRS = \
	src \
	include/robotDynamicsJRLJapan \
	doc \
	unitTesting

dynamicsJRLJapandir = $(libdir)/pkgconfig
dynamicsJRLJapan_DATA = dynamicsJRLJapan.pc

.PHONY: doc install-doc
doc install-doc:
	cd doc && ${MAKE} $@
