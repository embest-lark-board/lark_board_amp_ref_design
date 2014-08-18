I)
SD raw partition layout:

absolute address        iamge name        size               relative address to raw partition
     0x100000            preloader(s)   256KB(64KB/per)   ; offset is 0x00000
     0x140000            u-boot.img     256KB             ; offset is 0x40000
     0x200000            bm.bin         1MB							  ; offset is 0x100000
     0x300000            rbf.bin        14MB              ; offset is 0x200000


II)
make_sdimage_bm.sh:

it is bash script for generating the SD image. It comes from the altera's make_sdimage.sh
the usage example:

sudo ./make_sdimage_bm.sh -k zImage,socfpga_cyclone5.dtb -p preloader.bin -b u-boot.img -a bm.bin -f bm.rbf -r /home/xuewt/share/yocto/build/tmp/work/socfpga_cyclone5-poky-linux-gnueabi/altera-image-1.0-r0/rootfs/ -o sd_image_bm.bin

where:
-k : uImage and dtb
-p : preloader
-b : u-boot
-a : bm
-f : rbf
-r : the rootfs which is generated from the yocto
-o : the output image name

sd_image_bm.tar.bz2 is the prebuilt sd_image_bm.bin.

III)
update the preloader and u-boot.img partially:
preloader:
sudo dd if=preloader.bin of=/dev/sdb3 bs=64k seek=0;sudo sync
u-boot:
sudo dd if=u-boot.img of=/dev/sdb3 bs=64k seek=4;sudo sync

IV)
update the bm and rbf partially:
bm:
sudo dd if=bm.bin of=/dev/sdb3 bs=1M seek=1;sudo sync
rbf:
sudo dd if=bm.rbf of=/dev/sdb3 bs=2M seek=1;sudo sync


V)test
Kingston class 4 4GB:
  boot time: 49ms   load bm time: 23ms
  
Sony     class 10 16GB:
  boot time: 48ms      load bm time: 20ms
