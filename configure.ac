# versie hier doet er niet echt toe, is gerelateerd aan make dist
AC_INIT([cmux], [1.0], [wim.vinckier@powerdale.com])
LT_INIT
AM_INIT_AUTOMAKE([1.11 -Wall -Wno-extra-portability foreign])
AC_CONFIG_MACRO_DIR([m4])
AC_CHECK_LIB
AC_PROG_CC
# AC_PROG_RANLIB
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_FILES([Makefile])

AC_CHECK_LIB(gpio, gpioExport)
AC_CHECK_HEADER([gpio.h])

AC_OUTPUT
