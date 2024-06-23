##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  # Warning, if you want to debug the code, substitute -Os to -O0
  USE_OPT = -O0 -ggdb -lm -fomit-frame-pointer -falign-functions=16 -std=c11 -fsingle-precision-constant
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = no
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = 
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = no
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = yes
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0xA00
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x800
endif

# Enables the use of FPU on Cortex-M4 (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = dis_loc_dwm1001

# WARNING!
# In case of using eclipse, it is necessary that this variable contains the path
# to the path Distributed_localization_DWM1001 project. In case of executing make
# manually try 'makefile PROJECT_DIRECTORY=./'
#PROJECT_DIRECTORY ?= /home/developer/ChibiOS-eclipse-workspace/Distributed_localization_DWM1001

# Imported source files and paths
CHIBIOS         := ./ext/ChibiOS
CHIBIOS_CONTRIB := ./ext/ChibiOS-Contrib
CONFDIR         := ./cfg
BUILDDIR        := ./build
DEPDIR          := ./.dep
PSRCDIR        := ./src
PINCDIR        := ./include

# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files.
include $(CHIBIOS_CONTRIB)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_nrf52.mk
# HAL-OSAL files (optional).
include $(CHIBIOS_CONTRIB)/os/hal/hal.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/NRF5/NRF52832/platform.mk
include $(CHIBIOS_CONTRIB)/os/hal/boards/DWM1001-DEV/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
include $(CHIBIOS)/os/hal/lib/streams/streams.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk
# Other files (optional).
include $(CHIBIOS)/os/test/test.mk
include $(CHIBIOS)/test/rt/rt_test.mk
include $(CHIBIOS)/test/oslib/oslib_test.mk
include $(CHIBIOS)/os/various/shell/shell.mk

# Define linker script file here
LDSCRIPT= $(STARTUPLD_CONTRIB)/NRF52832.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC) \
       $(TESTSRC) \
       $(wildcard $(PSRCDIR)/*.c) \
	   $(CHIBIOS)/os/various/syscalls.c \

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC)

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C++ sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

INCDIR = $(CONFDIR) $(ALLINC) $(TESTINC) $(TESTHAL) $(PINCDIR)

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary
SREC = $(CP) -O srec

# ARM-specific options here
AOPT = 

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes  -Wno-unused-parameter 

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Wundef

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -DRADIO_ESB_PRIMARY_TRANSMITTER -DCHPRINTF_USE_FLOAT=1

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

# Terminal
TERM = konsole

#
# End of user defines
##############################################################################

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/
include $(RULESPATH)/rules.mk

OHEX = $(BUILDDIR)/$(PROJECT).hex
OELF = $(BUILDDIR)/$(PROJECT).elf
OBIN = $(BUILDDIR)/$(PROJECT).bin

include $(CHIBIOS_CONTRIB)/os/various/jlink.mk
include $(CHIBIOS_CONTRIB)/os/various/gdb.mk

.PHONY: openocd-flash openocd-debug-server openocd-debug

pin-reset: jlink-pin-reset
flash: all jlink-flash

# Gets the serial numbers of all DWM1001 boards and flashes each board
openocd-flash:
	$(shell export SERIAL_NUMBERS=`lsusb -v | grep -C 20 'SEGGER J-Link' | grep "iSerial" | awk '{print $$NF}'` ; \
	for serial_number in $$SERIAL_NUMBERS ; do \
		sed -i "s/adapter serial .*/adapter serial $$serial_number/g" openocd/flash_dwm1001.cfg ; \
		openocd -f openocd/flash_dwm1001.cfg ; \
	done)

debug: gdb-debug

# Opens a GDB sessions for each board connected
openocd-debug:
	$(shell export DEVICE_NUMBER=`lsusb -v | grep -C 20 'SEGGER J-Link' | grep "iSerial" | wc -l` ; \
	export OPENOCD_GDB_PORT=3333 ; \
	for i in `seq 1 $$DEVICE_NUMBER` ; do \
		sed -i "s/target extended-remote :.*/target extended-remote :$$((i+OPENOCD_GDB_PORT-1))/g" gdb/dwm1001.gdb ; \
		sleep 1 ; \
		$$TERM -e "arm-none-eabi-gdb -tui --command=./gdb/dwm1001.gdb" & \
		sleep 1 ; \
	done)

erase-all: jlink-erase-all
debug-server: jlink-debug-server

# Opens a openocd GDB server for each board connected
### SERVER IS RUN IN PARALLEL SO CTRL+C MAY NOT KILL THE PROCESS AND KEEP OPENOCD USED, pkill openocd can be used
openocd-debug-server:
	$(shell export SERIAL_NUMBERS=`lsusb -v | grep -C 20 'SEGGER J-Link' | grep "iSerial" | awk '{print $$NF}'` ; \
	export COUNTER=0 ; \
	export OPENOCD_GDB_PORT=3333 ; \
	export OPENOCD_TELNET_PORT=4444 ; \
	export OPENOCD_TCL_PORT=6666 ; \
	for serial_number in $$SERIAL_NUMBERS ; do \
		sed -i "s/adapter serial .*/adapter serial $$serial_number/g" openocd/dwm1001.cfg ; \
		sed -i "s/gdb_port .*/gdb_port $$((COUNTER+OPENOCD_GDB_PORT))/g" openocd/dwm1001.cfg ; \
		sed -i "s/telnet_port .*/telnet_port $$((COUNTER+OPENOCD_TELNET_PORT))/g" openocd/dwm1001.cfg ; \
		sed -i "s/tcl_port .*/tcl_port $$((COUNTER+OPENOCD_TCL_PORT))/g" openocd/dwm1001.cfg ; \
		if [[ $$COUNTER -eq 0 ]]; then \
			openocd -f openocd/dwm1001.cfg & \
		else \
			$$TERM -e "openocd -f openocd/dwm1001.cfg" & \
		fi ; \
		sleep 1 ; \
		COUNTER=$$((COUNTER+1)) ; \
	done)
