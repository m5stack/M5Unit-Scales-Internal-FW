USER_SOURCES = $(wildcard Core/user/*.c)
USER_SOURCES += $(wildcard Core/rtt/*.c)

DEBUG = 0
OPT = -Os

include Makefile
