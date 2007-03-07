
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

AC_ARG_WITH(T3D,
	for using T3D V3D and R3D as SmallMatrix and SmallVector,
	WITH_T3D="1"
	echo "Using t3d"
	if test "x${withval}"="x" ; then
		T3D_PKG=1
		echo "Using pkg-config for t3d"
	else
		T3D_PKG=0
		T3D_PREFIX="${withval}"
		echo "t3d is at ${withval}"
	fi
	,
	WITH_T3D="0"
	echo "Not using t3d"
	)

if test "$WITH_T3D" = "1" ; then
	echo "Using pkg-config for t3d"
	if test "$T3D_PKG" = "1" ; then
		echo "Using pkg-config for t3d"
		PKG_CHECK_MODULES(T3D, t3d)
		AC_SUBST(T3D_CFLAGS)
		AC_SUBST(T3D_LIBS)
	else
		T3D_CFLAGS="-I${T3D_PREFIX}/include"
		T3D_LIBS="-L${T3D_PREFIX}/lib -lt3d"
	fi
	CXXFLAGS="${CXXFLAGS} ${T3D_CFLAGS} -D_T3D_R3D_V3D_"
	LDFLAGS="${LDFLAGS} ${T3D_LIBS}"
fi

AC_OUTPUT(
	dynamicsJRLJapan.pc
	Makefile
	include/robotDynamicsJRLJapan/Makefile
	src/Makefile
	doc/Makefile
	doc/package.dox
	unitTesting/Makefile
	)

