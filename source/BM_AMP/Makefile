BUILD_TOOL_CROSS := arm-none-eabi
#BUILD_TOOL_ROOT := C:/Program\ Files\ (x86)/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI
BUILD_TOOL_ROOT := C:/Program\ Files\ (x86)/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI
#BUILD_TOOL_ROOT := C:/altera/13.1/embedded/host_tools/mentor/gnu/arm/baremetal

BM_ROOT := $(CURDIR)

BM_SRCDIR           := $(BM_ROOT)/AMP_BM_REF_DESIGN
BM_LINKER_SCRIPT := $(BM_ROOT)

CROSS_COMPILE := $(BUILD_TOOL_CROSS)-

BM_OBJS := AMP_BM_REF_DESIGN/main.o AMP_BM_REF_DESIGN/exception.o AMP_BM_REF_DESIGN/gic.o AMP_BM_REF_DESIGN/mmu.o AMP_BM_REF_DESIGN/memset_int.o AMP_BM_REF_DESIGN/iic_bitbang.o AMP_BM_REF_DESIGN/start.o
BM_OBJS +=	AMP_BM_REF_DESIGN/dma.o AMP_BM_REF_DESIGN/dma_internal.o AMP_BM_REF_DESIGN/reset_manager.o AMP_BM_REF_DESIGN/fpga_manager.o
BM_OBJS +=	AMP_BM_REF_DESIGN/bmlog.o AMP_BM_REF_DESIGN/twd.o AMP_BM_REF_DESIGN/l2lock.o AMP_BM_REF_DESIGN/cache.o
BM_OBJS += AMP_BM_REF_DESIGN/qspi.o
BM_OBJS += AMP_BM_REF_DESIGN/boot_mode.o AMP_BM_REF_DESIGN/sd_load.o

all: bm.axf Makefile
	@echo BM_ROOT:$(BM_ROOT)
	@echo BM_SRCDIR:$(BM_SRCDIR)
	@echo BM_LINKER_SCRIPT:$(BM_LINKER_SCRIPT)

bm.axf: bm.lds $(BM_OBJS) 
	$(LD) -o bm.axf $(LDFLAGS)  -Xlinker --script=bm.lds $(BM_OBJS) $(GCC_LIBS) -nostdlib -Xlinker -Map=bm.map    
	$(OBJCOPY) -I elf32-little -O binary $@ bm.bin
                           
build: build-bm 

build-bm: $(BM_OBJS)

.PHONY: all build build-bm clean

#Do Not allow gnu make implicit rules
.SUFFIXES:

AS		= $(BUILD_TOOL_ROOT)/bin/$(CROSS_COMPILE)gcc
LD		= $(BUILD_TOOL_ROOT)/bin/$(CROSS_COMPILE)gcc
CC		= $(BUILD_TOOL_ROOT)/bin/$(CROSS_COMPILE)gcc 
AR		= $(BUILD_TOOL_ROOT)/bin/$(CROSS_COMPILE)ar
RM              = $(BUILD_TOOL_ROOT)/bin/cs-rm
OBJCOPY = $(BUILD_TOOL_ROOT)/bin/$(CROSS_COMPILE)objcopy


NM      = echo unknown NM
LDR     = echo unknown LDR
STRIP   = echo unknown STRIP
RANLIB  = echo unknown RANLIB
MKDIR	= mkdir
MV		= move

INCDIRS = $(BM_SRCDIR)
BUILD_DEBUG = 0

ifeq ($(BUILD_DEBUG),1)
BUILD_DEFINES	= -D__DEBUG__
OPT_DEFINES		= -O0
DEBUG_TARGET	=  debug
else
BUILD_DEFINES	= -DRELEASE__=1
OPT_DEFINES		= -O2
#OPT_DEFINES		= -Os
DEBUG_TARGET	=  release
endif
BINARY_DIR_ROOT	:= $(DEBUG_TARGET)

BUILD_TARGET    := ROM
ifeq ($(MAKECMDGOALS),codelink)
  BUILD_TARGET      := codelink
endif
ifeq ($(MAKECMDGOALS),pluto)
  BUILD_TARGET 			:= pluto
endif
BINARY_DIR_ROOT := $(BINARY_DIR_ROOT)/$(BUILD_TARGET)


BUILDFLAGS = -DBUILD_TARGET=$(BUILD_TARGET) -DBUILD_TARGET_$(BUILD_TARGET)

#PROCESSOR = -march=armv7-a -mtune=cortex-a9 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard
PROCESSOR = -march=armv7-a -mtune=cortex-a9 -mcpu=cortex-a9

CCFLAGS = --verbose -g -c -fno-builtin-printf -pipe -ftree-vectorize -ffast-math -funsafe-math-optimizations -fsingle-precision-constant -ffreestanding -marm -Wall -Wstrict-prototypes -fno-stack-protector -isystem 
CCFLAGS += -I$(BUILD_TOOL_ROOT)/lib/gcc/$(BUILD_TOOL_CROSS)/4.6.3/include

CFLAGS = -g -D__ARM__ -DCONFIG_ARM -D__KERNEL__ -DHAVE_HPS_BASE_ADDRS_H -DHAVE_HPS_CLOCK_MANAGER_REGS_H -DHAVE_HPS_DW_APB_TIMERS_H -DHAVE_HPS_DW_APB_UART_H -DHAVE_HPS_INTERRUPTS_H -DHAVE_HPS_RESET_MANAGER_REGS_H -DCONFIG_ZERO_BSS -DCONFIG_ENABLE_NEON -DPLATFORM_DEV5XS1 -DEXP_MIDR_VAL=0x412FC092  -DINLINE=inline  

ASFLAGS =  -g -c -march=armv7-a -mtune=cortex-a9 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard -D__ASSEMBLY__ 
#ASFLAGS =  -g -c -march=armv7-a -mtune=cortex-a9 -mcpu=cortex-a9 -D__ASSEMBLY__
ASFLAGS += $(CCFLAGS) $(CFLAGS)
DEP_FLAGS = -MD
CFLAGS += -DPLATFORM_DEV5XS1

CCFLAGS += $(BUILD_DEFINES)
CCFLAGS += $(OPT_DEFINES)
CCFLAGS	+= $(PROCESSOR) $(CFLAGS)
CCFLAGS += $(BUILDFLAGS)


LDFLAGS = 

MAIN_LIBS = 

OBJ_DIR_ROOT			:= OBJS/$(BINARY_DIR_ROOT)
OBJ_DIR					:= $(OBJ_DIR_ROOT)
MAIN_LIB_ROOT			:= $(MAIN_DIR)/lib
MAIN_LIB_BINARY_ROOT	:= $(MAIN_LIB_ROOT)/$(BINARY_DIR_ROOT)

GCC_LIBS := -L$(BUILD_TOOL_ROOT)/lib/gcc/$(BUILD_TOOL_CROSS)/FP/lib -lm -lc -L$(BUILD_TOOL_ROOT)/lib/gcc/$(BUILD_TOOL_CROSS)/4.6.3 -lgcc

ONCHIP_ROM_BASE_ADDRESS	:=0x00000000
ONCHIP_RAM_BASE_ADDRESS	:=0x00008000
ONCHIP_ROM_LENGTH		:=0x00008000
ONCHIP_RAM_LENGTH		:=0x00008000

DUMMY_ADDRESS			:=0xFFFF0000

%.o: %.S
	$(AS) $(ASFLAGS) $(INCDIRS:%=-I%)  -o $@ $<

%.o: %.c
	$(CC) $(CCFLAGS) $(INCDIRS:%=-I%) $(DEP_FLAGS) -o $@ $<

$(OBJ_DIR)/%.o: %.S
	$(AS) $(ASFLAGS) $(INCDIRS:%=-I%) $(DEP_FLAGS) $(BUILDFLAGS) -o $@ $<

$(OBJ_DIR)/%.o: %.c
	$(CC) $(CCFLAGS) $(INCDIRS:%=-I%) $(DEP_FLAGS) $(BUILDFLAGS) -o $@ $<

$(OBJ_DIR):
	$(MKDIR) $(OBJ_DIR)

.PHONY: clean clean-common
clean: clean-common

DEP_FILES := $(patsubst %.o,%.d,$(BM_OBJS))

clean-common:
	$(RM) $(BM_OBJS) $(DEP_FILES) bm.axf bm.bin -f

-include $(DEP_FILES)


