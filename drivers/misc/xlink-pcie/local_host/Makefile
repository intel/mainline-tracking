ccflags-y += -Wall -Wno-unused-function -Werror -DXLINK_PCIE_LOCAL=1
ccflags-y += -I$(srctree)/drivers/pci/controller/dwc

obj-$(CONFIG_XLINK_PCIE_LH_DRIVER) += mxlk_ep.o
mxlk_ep-objs := epf.o
mxlk_ep-objs += core.o
mxlk_ep-objs += dma.o
mxlk_ep-objs += ../common/if.o
mxlk_ep-objs += ../common/util.o
