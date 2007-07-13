# 
#  Copyright 
# 


jrlCIRdynamicsJRLJapandir = $(datadir)/doc/jrlCIRdynamicsJRLJapan

EXTRA_DIST = \
	header.html \
	footer.html \
	package.css \
	pictures/footer.txt \
	additionalDoc/package.h 


install: html-doc
	mkdir -p $(jrlCIRdynamicsJRLJapandir)
	$(INSTALL_DATA) html/* $(jrlCIRdynamicsJRLJapandir)
	mkdir -p $(jrlCIRdynamicsJRLJapandir)/pictures
	$(INSTALL_DATA) $(srcdir)/pictures/* $(jrlCIRdynamicsJRLJapandir)/pictures
	mkdir -p $(datadir)/doc/doxytag
	$(INSTALL_DATA) dynamicsJRLJapan.doxytag $(datadir)/doc/doxytag

html-doc: package.dox
	doxygen $<

clean-local:
	-rm -r html
