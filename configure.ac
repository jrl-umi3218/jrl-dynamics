
dnl 
dnl configure.ac file for package jrlCIRdynamicsJRLJapan 
dnl
AC_PREREQ(2.59)

define([dynamicsJRLJapan_version],[1.6.99])
dnl define([dynamicsJRLJapan_version], [esyscmd([/usr/bin/git log -n 1 | /bin/grep commit | /usr/bin/awk '{print $2}' | /usr/bin/git describe | /usr/bin/tr -d '\n'])])

AC_INIT([dynamicsJRLJapan], [dynamicsJRLJapan_version], [olivier.stasse@aist.go.jp])
undefine([dynamicsJRLJapan_version])
AM_INIT_AUTOMAKE([foreign no-define])

AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_LIBTOOL

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
