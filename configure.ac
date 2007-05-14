
dnl 
dnl configure.ac file for package jrlCIRdynamicsJRLJapan 
dnl

AC_INIT([dynamics-JRLJapan], [1.1], [olivier.stasse@aist.go.jp])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE

AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_LIBTOOL

dnl Dynamic Robot Abstract Interface
PKG_CHECK_MODULES(ROBOTDYNAMICS, [abstractRobotDynamics >= 1.2])
AC_SUBST(ROBOTDYNAMICS_CFLAGS)
AC_SUBST(ROBOTDYNAMICS_LIBS)

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
