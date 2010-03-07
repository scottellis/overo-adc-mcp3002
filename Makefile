# Makefile for mcp3002 ADC kernel module

ifneq ($(KERNELRELEASE),)
    obj-m := adc.o
else
    PWD := $(shell pwd)

default:
ifeq ($(strip $(KERNELDIR)),)
	$(error "KERNELDIR is undefined!")
else
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules 
endif


clean:
	rm -rf *~ *.ko *.o *.mod.c modules.order Module.symvers .adc* .tmp_versions

endif

