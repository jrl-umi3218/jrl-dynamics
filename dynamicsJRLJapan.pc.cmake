# Copyright 2008, 2009, 2010, 
#
# Francois Keith
# Florent Lamiraux
# Olivier Stasse
#
# JRL/LAAS, CNRS/AIST
#
# This file is part of dynamicsJRLJapan.
# dynamicsJRLJapan is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# dynamicsJRLJapan is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with dynamicsJRLJapan.  If not, see <http://www.gnu.org/licenses/>.
#
prefix=${CMAKE_INSTALL_PREFIX}
exec_prefix=${install_pkg_prefix}/bin
libdir=${install_pkg_prefix}/lib
includedir=${install_pkg_prefix}/include
datarootdir=${install_pkg_prefix}/share
docdir=${install_pkg_datarootdir}/doc/${PROJECT_NAME}

Name: ${PROJECT_NAME}
Description: ${PROJECT_DESCRIPTION}
Version: ${PROJECT_VERSION}
Requires: ${PROJECT_REQUIREMENTS}
Libs: ${LIBDIR_KW}${install_pkg_libdir} ${${PROJECT_NAME}_LDFLAGS}
Cflags: -I${install_pkg_include_dir} ${${PROJECT_NAME}_export_CXXFLAGS}

