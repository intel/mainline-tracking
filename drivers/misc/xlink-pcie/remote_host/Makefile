ccflags-y += -DCONFIG_ARCH_THUNDERBAY=1 -DXLINK_PCIE_REMOTE=1

obj-$(CONFIG_XLINK_PCIE_RH_DRIVER) += mxlk.o
mxlk-objs := main.o
mxlk-objs += pci.o
mxlk-objs += core.o
mxlk-objs += boot.o
mxlk-objs += ../common/util.o
mxlk-objs += ../common/interface.o
