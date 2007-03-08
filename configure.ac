
dnl 
dnl configure.ac file for package jrlCIRdynamicsJRLJapan 
dnl

AC_INIT([dynamics-JRLJapan], [1.0], [olivier.stasse@aist.go.jp])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE

AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_LIBTOOL

PKG_CHECK_MODULES(MAL, MatrixAbstractLayer)
AC_SUBST(MAL_CFLAGS)

PKG_CHECK_MODULES(ROBOTDYNAMICS, abstractRobotDynamics)
AC_SUBST(ROBOTDYNAMICS_CFLAGS)

AC_SUBST(LDFLAGS)

AC_ARG_WITH(boost,
        for ublas,
        BOOST_CFLAGS="-I${withval}/include -D_BOOST_MATRIX_"
        BOOST_LDFLAGS="-llapack"
        CXXFLAGS="${CXXFLAGS} ${BOOST_CFLAGS}"
	LDFLAGS="${LDFLAGS} ${BOOST_LDFLAGS}",
        )

AC_ARG_WITH(boost-sandbox,
        for lapack bindings,
        BOOST_SANDBOX_CFLAGS="-I${withval} -D_BOOST_MATRIX_"
        CXXFLAGS="${CXXFLAGS} ${BOOST_SANDBOX_CFLAGS}",
        )

AC_ARG_WITH(VNL,
        for using VNL linear algebra package,
        VNL_CFLAGS="-I${withval}/include -D_VNL_MATRIX_"
        VNL_LDFLAGS="-L${withval}/lib -lVNL -llapack"
        CXXFLAGS="${CXXFLAGS} ${VNL_CFLAGS}"
        LDFLAGS="${LDFLAGS} ${VNL_LDFLAGS}"
        )

dnl
dnl If configure is invoked --with-t3d without path, call pkg-config
dnl
AC_ARG_WITH(t3d,
	AC_HELP_STRING([--with-t3d],[for using T3D V3D and R3D as SmallMatrix and SmallVector]))
if test x$with_t3d = xno ; then
	AC_MSG_NOTICE([not using T3D package])
	T3D_REQUIRES=
	T3D_CFLAGS=
	T3D_LIBS=
else
	if test x$with_t3d = xyes ; then
		T3D_REQUIRES="t3d >= 2.1,"
		PKG_CHECK_MODULES(T3D, [t3d >= 2.1])
	else
		T3D_CFLAGS="-I${with_t3d}/include"
		T3D_LIBS="-L${with_t3d}/lib -lt3d"
		AC_MSG_NOTICE([using T3D package at ${with_t3d}])
	fi
	CXXFLAGS="${CXXFLAGS} ${T3D_CFLAGS} -D_T3D_R3D_V3D_"
	LDFLAGS="${LDFLAGS} ${T3D_LIBS}"
fi

AC_SUBST(T3D_CFLAGS)
AC_SUBST(T3D_LIBS)
AC_SUBST(T3D_REQUIRES)

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

