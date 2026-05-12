# 01-Beginners-guide/03-Build-your-first-project#optional-source-files
#git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git $sysroot
#sysroot='rtos_sysroot' #repos_root dir

#mkdir freertos (gitignored) && cp this.file freertos && sh this.file TODO into Makefile
sysroot="$HOME/codings/external_libs/rtos-kernel"
dest='./' #no arg, just copy over here
compiler='GCC' #arg
mcu='ARM_CM4F' #arg
ports="${sysroot}/portable/${compiler}/${mcu}/"
mem_mang='portable/MemMang/' # no arg, just cp all c files
incl='include/'


cp $sysroot/*.c $dest
cp $sysroot/$mem_mang*.c $dest
cp $sysroot/$incl*.h $dest
cp $ports* $dest

echo "00a1 ¡ done!¡"
