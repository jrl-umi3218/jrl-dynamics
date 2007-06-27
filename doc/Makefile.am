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


install-doc: 
	mkdir -p $(jrlCIRdynamicsJRLJapandir)
	$(INSTALL_DATA) html/* $(jrlCIRdynamicsJRLJapandir)
	mkdir -p $(jrlCIRdynamicsJRLJapandir)/pictures
	$(INSTALL_DATA) $(srcdir)/pictures/* $(jrlCIRdynamicsJRLJapandir)/pictures

doc: package.dox
	doxygen $<

clean-local:
	-rm -r html
