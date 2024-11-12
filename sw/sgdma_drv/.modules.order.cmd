cmd_/home/hge/work/sw_1ch/sgdma_drv/modules.order := {   echo /home/hge/work/sw_1ch/sgdma_drv/anlogic_pci.ko; :; } | awk '!x[$$0]++' - > /home/hge/work/sw_1ch/sgdma_drv/modules.order
