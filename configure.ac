
dnl 
dnl configure.ac file for package jrlCIRdynamicsJRLJapan 
dnl

AC_INIT([dynamicsJRLJapan], [1.4], [olivier.stasse@aist.go.jp])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE

AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_LIBTOOL

dnl Dynamic Robot Abstract Interface
PKG_CHECK_MODULES(ROBOTDYNAMICS, [abstractRobotDynamics >= 1.5])
ROBOTDYNAMICS_PREFIX=`$PKG_CONFIG abstractRobotDynamics --variable=prefix`
AC_SUBST(ROBOTDYNAMICS_CFLAGS)
AC_SUBST(ROBOTDYNAMICS_LIBS)
AC_SUBST(ROBOTDYNAMICS_PREFIX)

dnl Create configuration files
AC_OUTPUT(
	dynamicsJRLJapan.pc
	Makefile
	include/robotDynamicsJRLJapan/Makefile
	src/Makefile
	doc/Makefile
	doc/package.dox
	unitTesting/Makefile
	)
