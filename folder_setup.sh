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

PROJECT_NAME='' #
# check variable set, error if not & message
libopencm3_sources='' #'/external_libs/libopencm3/'
# check exist, error if not & message
rtos_sources='' #'/external_libs/rtos-kernel/'
# check exist, error if not & message
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

: << 'MAX3232'

             ┌─────────────────────────┐
             │                         │
gnd 1   o────┤                         ├────o  8 ->
vcc 2   o────┤     ┌─────────────┐     ├────o  7 <-
->  3   o────┤     │   MAX3232   │     ├────o  6 vcc
<-  4   o────┤     │'            │     ├────o  5 gnd
             │     └─────────────┘     │
             │                         │
             └─────────────────────────┘
        RS232 side                    TTL side
module.pin:| pin max3232 | picto | where to connect
1:              15         -        gnd   (TRS.sleeve)
2:              16         +    vcc ¡!!NC!!¡ 
3:              8          ->   TX device (-5,61V) (TRS.Ring)
4:              7          <-   RX device (0V)     (TRS.Tip)

5:              15          -   gnd mcu.gnd
6:              16          +   mcu.3.3v
7:              10          <-  mcu.tx (uart2.a2) 
8:              9           ->  mcu.rx (uart2.a3)

vcc from mcu(3.3V)
gnd one from mcu, additional from RS232 device (TRS.sleeve)

MAX3232
