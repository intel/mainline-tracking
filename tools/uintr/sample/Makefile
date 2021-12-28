CC		= $(CROSS_COMPILE)gcc
BUILD_OUTPUT	:= $(CURDIR)
PREFIX		?= /usr
DESTDIR		?=

ifeq ("$(origin O)", "command line")
	BUILD_OUTPUT := $(O)
endif

uipi_sample : uipi_sample.c

CFLAGS +=       -Wall -static -muintr -mgeneral-regs-only -minline-all-stringops
LDFLAGS +=      -lpthread
%: %.c
	@mkdir -p $(BUILD_OUTPUT)
	$(CC) $(CFLAGS) $< $(LDFLAGS) -o $(BUILD_OUTPUT)/$@

all: uipi_sample

.PHONY : clean
clean :
	@rm -f $(BUILD_OUTPUT)/uipi_sample
