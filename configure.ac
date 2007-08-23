
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

AC_SUBST(BOOST_PATH,	[])

AC_ARG_WITH(boost, AC_HELP_STRING([--with-boost],
        [use boost for generic NxP matrix implementation]),,with_boost=no)

if test x${with_boost} = xno ; then
	AC_ERROR([please configure --with-boost])
fi

dnl Create configuration files
AC_OUTPUT(
	dynamicsJRLJapan.pc
	Makefile
	include/robotDynamicsJRLJapan/Makefile
	include/Makefile
	src/Makefile
	doc/Makefile
	doc/package.dox
	unitTesting/Makefile
	)
