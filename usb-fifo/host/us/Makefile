BINNAME = host

PREFIX	:=
CC		:= $(PREFIX)gcc
LD		:= $(PREFIX)ld
DISASM  := $(PREFIX)objdump
BINARY	:= $(PREFIX)objcopy
RMf		= rm -f

CC_COMPILER	:=
CC_HOST		:=
CC_WARNINGS	:= -Wall
CC_INCLUDE	:= -I/usr/include/libusb-1.0
CC_DEBUG	:=

CC_LIST 	=

CC_FLAGS	= $(CC_HOST) $(CC_WARNINGS) $(CC_INCLUDE) $(CC_DEBUG)
CFLAGS		= $(CC_FLAGS) $(CC_LIST) $(CC_COMPILER)

LDFLAGS		= -lusb-1.0

OTarget	    := $(BINNAME)

SOURCEFILES :=  *.c Makefile


.PHONY: default
.PHONY: clean

all: host.o

host.o: host.c Makefile
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(BINNAME) host.c

clean:
	$(RM) *.o $(BINNAME)