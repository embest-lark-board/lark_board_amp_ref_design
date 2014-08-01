
1)BM_AMP is soource code for bare metal, and you should compile it in  the windows, and you may
  refer to the tool diretory.
  
2)linux-3.10-ltsi bases on the https://github.com/embest-lark-board/linux-3.10-ltsi which is 
  normal for the lark board.
  
  compile steps:
  make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-  LOADADDR=0x8000 socfpga_amp_defconfig
  make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-  LOADADDR=0x8000 uImage 
  make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-  LOADADDR=0x8000 dtbs
  
3)u-boot-2013-lark-board-amp bases on the https://github.com/embest-lark-board/u-boot-2013-lark-board
  which is normal for  the lark board.
  
  compile steps:
  make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- socfpga_cyclone5_config
  make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- all

