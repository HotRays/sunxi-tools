#CROSS_COMPILE ?= aarch64-linux-gnu-gcc
CROSS_COMPILE =  arm-linux-gnueabihf-gcc
OBJH := $(wildcard *.h)
#VPATH = ../include
#COMPILE_FLAG = -nostdlib -mabi=ilp32 -T emmc.lds -Wl,-N
COMPILE_FLAG += -Os -fpic -Wall -nostdlib -T emmc.lds -Wl,-N
COMPILE_FLAG += -fno-common -fno-builtin -ffreestanding -nostdinc -fno-strict-aliasing
COMPILE_FLAG += -fno-stack-protector -fno-toplevel-reorder
COMPILE_FLAG += -Wstrict-prototypes -Wno-format-nonliteral -Wno-format-security
COMPILE_FLAG += -Wa,-mimplicit-it=thumb
#emmc-write.bin:mmc.o mmc_bsp.o 
#	$(CROSS_COMPILE) $(COMPILE_FLAG) -o $@ $^
#mmc_bsp.o:mmc_bsp.c $(OBJH)
#	$(CROSS_COMPILE) $(COMPILE_FLAG) $^ -o $@
#mmc.o:mmc.c $(OBJH)
#	$(CROSS_COMPILE) $(COMPILE_FLAG) $^ -o $@
emmc-write.bin:mmc.c mmc_bsp.c $(OBJH)
	$(CROSS_COMPILE) $(COMPILE_FLAG) $^ -o $@
.PHONY : clean
clean:
	@-rm -r *.o emmc-write.bin
