#lib_LTLIBRARIES = libgpio.la
#libgpio_la_SOURCES = gpio.c
#include_HEADERS = gpio.h
## see https://www.gnu.org/software/libtool/manual/html_node/Updating-version-info.html
#libgpio_la_LDFLAGS = -version-info 0:1:0
#ACLOCAL_AMFLAGS = -I m4
bin_PROGRAMS = cmux
cmux_SOURCES = cmux.c
cmux_CFLAGS = $(LIBGPIOCFLAGS) $(AM_CFLAGS) $(GPIO_FLAGS)
cmux_LDFLAGS = $(LIBGPIOLDFLAGS) $(GPIO_LIBS) 
# http://www.gnu.org/software/automake/manual/html_node/Program-and-Library-Variables.html
# cmux_DEPENDENCIES = libgpio
cmux_LDDADD = libgpio.so

