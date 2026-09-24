# 01-Beginners-guide/03-Build-your-first-project#optional-source-files
#git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git $sysroot
#sysroot='rtos_sysroot' #repos_root dir

#mkdir freertos (gitignored) && cp this.file freertos && sh this.file TODO into Makefile
sysroot="/external_libs/rtos-kernel"
dest='./' #no arg, just copy over here
compiler='GCC' #arg
mcu='ARM_CM3'  #'ARM_CM4F' #arg
ports="${sysroot}/portable/${compiler}/${mcu}/"
mem_mang='portable/MemMang/' # no arg, just cp all c files
incl='include/'


cp $sysroot/*.c $dest
cp $sysroot/$mem_mang*.c $dest
cp $sysroot/$incl*.h $dest
cp $ports* $dest

echo "00a1 ¡ done!¡"

opencm3="
#include \"FreeRTOS.h\"
#include \"task.h\"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
extern void vPortSVCHandler( void ) __attribute__ (( naked ));
extern void xPortPendSVHandler( void ) __attribute__ (( naked ));
extern void xPortSysTickHandler( void );
void sv_call_handler(void){vPortSVCHandler();}
void pend_sv_handler(void){xPortPendSVHandler();}
void sys_tick_handler(void){xPortSysTickHandler();}
/* end opncm3.c */"
#kudos to ve3wwg

#echo -n "${opencm3}" > opencm3.c

rtos_sources=''
compiler_dir='GCC' #wont change, cuz gcc exclusively usage
mcu_dir='ARM_CM3' # f103 'ARM_CM4F' #f405 ? f706zit6

if [[ -d "$folder" ]]; then
    echo "Folder exists"
fi
[[ -d "$folder" ]] && echo "Exists" || echo "Doesn't exist"
#shell
if [ -d "$folder" ]; then
    echo "Folder exists"
fi
[ -d "$folder" ] && echo "Exists"

if [ -n "$var" ]; then
    echo "var is not empty"
fi

[ -z "$var" ]   # true if var is empty

[ -f "$file" ] # file?

[ -e "$path" ] # For checking whether a path exists at all (file, directory, symlink, etc.)


