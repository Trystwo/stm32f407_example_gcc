##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [4.2.0-B44] date: [Sat Dec 30 18:03:58 CST 2023] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------



######################################
# target
######################################
TARGET = stm32f407xx
######################################
# OS
######################################
#OS = $(shell uname -s)
######################################
# FIND
######################################
ifeq ($(OS),Windows_NT)
	FIND = E:/msys64/usr/bin/find.exe
else ifeq ($(OSTYPE), msys)
	FIND = E:/msys64/usr/bin/find.exe
else
	FIND = find
endif
######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
SOURCES = \
Core/Src \
Drivers/BSP \
Drivers/SYSTEM \
Drivers/STM32F4xx_HAL_Driver/Src \
Drivers/STM32F4xx_HAL_Driver/Src/Legacy \
Middlewares/LVGL/GUI_APP \
Middlewares/LVGL/GUI/lvgl \
Middlewares/FreeRTOS \
Middlewares/MALLOC 

SOURCES_DIRS := $(shell $(FIND) $(SOURCES) -type d)
C_SOURCES=$(foreach dir, $(SOURCES_DIRS), $(wildcard $(dir)/*.c))

# ASM sources
ASM_SOURCES =  \
startup_stm32f407xx.s

# ASM sources
ASMM_SOURCES = 


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F407xx


# AS includes
AS_INCLUDES = 

# C includes
INCLUDES =  \
Core/Inc \
Drivers/BSP/LED \
Drivers/BSP/SRAM \
Drivers/BSP/LCD \
Drivers/SYSTEM/sys \
Drivers/SYSTEM/usart \
Drivers/SYSTEM/delay \
Drivers/STM32F4xx_HAL_Driver/Inc \
Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
Drivers/CMSIS/Device/ST/STM32F4xx/Include \
Drivers/CMSIS/Include \
Drivers/BSP/24CXX \
Drivers/BSP/IIC \
Drivers/BSP/KEY \
Drivers/BSP/TIMER \
Drivers/BSP/TOUCH \
Middlewares/LVGL/GUI/lvgl \
Middlewares/FreeRTOS/include \
Middlewares/MALLOC \
Middlewares/FreeRTOS/portable/GCC/ARM_CM4F \
Middlewares/LVGL/GUI/lvgl/examples/porting \
Middlewares/LVGL/GUI_APP/demos/stress \
Middlewares/LVGL/GUI_APP/demos/music \
Middlewares/LVGL/GUI_APP/ui \
Middlewares/LVGL/GUI_APP/ui/components

INC_DIR=$(foreach dir, $(INCLUDES), $(wildcard $(dir)/*.h))
C_INCLUDES=$(patsubst %,-I%, $(INCLUDES))

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F407ZGTx_FLASH.ld

# libraries 浮点数打印-u _printf_float
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -u _printf_float -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASMM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASMM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile $(INC_DIR)
	@echo 'CC  $<'
	@mkdir -p $(dir $@)
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(<:.c=.lst) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo 'AS  $<'
	@$(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	@echo 'AS  $<'
	@$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@echo 'LD  $@'
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo 'HEX $@'
	@$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo 'BIN $@'
	@$(BIN) $< $@	
	@echo 'SZ  $<'
	@$(SZ) $<

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

print:
	echo INC_DIR $(INC_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)
#######################################
# flash
#######################################
link.cfg = stlink.cfg
mcu.cfg = stm32f4x.cfg
flash: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/$(link.cfg) -f target/$(mcu.cfg) -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"
# *** EOF ***
