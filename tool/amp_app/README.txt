linux sample app use to inspect linux/bm share memory region in user space.

how to make:
arm-linux-gnueabihf-gcc amp_app.c -o amp_app



usage:
 amp_app /dev/amp 504365056 4096
where:504365056 (dec) = 0x1e100000(hex)
      4096 is memory size.
