#!/bin/sh
aclocal
autoheader
libtoolize -c -f
automake -ca
autoconf
