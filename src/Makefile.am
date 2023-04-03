# Note: plugindir is set in configure

##############################################################################
# TODO: change libgstplugin.la to something else, e.g. libmysomething.la     #
##############################################################################
plugin_LTLIBRARIES = libgstautofocus.la

##############################################################################
# TODO: for the next set of variables, name the prefix if you named the .la, #
#  e.g. libmysomething.la => libmysomething_la_SOURCES                       #
#                            libmysomething_la_CFLAGS                        #
#                            libmysomething_la_LIBADD                        #
#                            libmysomething_la_LDFLAGS                       #
##############################################################################

## Plugin 1

# sources used to compile this plug-in
libgstautofocus_la_SOURCES = gstautofocus.c gstautofocus.h autofocusControl.c autofocusControl.h i2c_control.c i2c_control.h i2c.c i2c.h logger.c logger.h

# compiler and linker flags used to compile this plugin, set in configure.ac
libgstautofocus_la_CFLAGS = $(GST_CFLAGS) -Wall -Wextra -Wpedantic -O3
libgstautofocus_la_LIBADD = $(GST_LIBS)
libgstautofocus_la_LDFLAGS = $(GST_PLUGIN_LDFLAGS)
libgstautofocus_la_LIBTOOLFLAGS = --tag=disable-static

# headers we need but don't want installed
noinst_HEADERS = gstautofocus.h
