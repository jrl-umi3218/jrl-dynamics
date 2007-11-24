
dnl 
dnl configure.ac file for package jrlCIRdynamicsJRLJapan 
dnl
AC_PREREQ(2.59)

AC_INIT([dynamicsJRLJapan], [1.6.99], [olivier.stasse@aist.go.jp])
AM_INIT_AUTOMAKE([foreign no-define])

AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_LIBTOOL

dnl Matrix Abstraction Layer
define(mal_reqd, [MatrixAbstractLayer >= 1.3])
AC_SUBST(MAL_REQD, "mal_reqd")
PKG_CHECK_MODULES(MAL, mal_reqd)
AC_SUBST(MAL_CFLAGS)
AC_SUBST(MAL_LIBS)

dnl Dynamic Robot Abstract Interface
define(robotdynamics_reqd, [abstractRobotDynamics >= 1.7])
AC_SUBST(ROBOTDYNAMICS_REQD, "robotdynamics_reqd")
PKG_CHECK_MODULES(ROBOTDYNAMICS, robotdynamics_reqd)
ROBOTDYNAMICS_PREFIX=`$PKG_CONFIG abstractRobotDynamics --variable=prefix`
AC_SUBST(ROBOTDYNAMICS_PREFIX)

dnl Create configuration files
AC_OUTPUT(
	dynamicsJRLJapan.pc
	Makefile
	include/Makefile
	src/Makefile
	doc/Makefile
	doc/package.dox
	unitTesting/Makefile
	)
