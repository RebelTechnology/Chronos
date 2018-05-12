# Library path
LIBROOT=$(TEMPLATEROOT)/Drivers

# Build path
BUILD=$(TEMPLATEROOT)/Build

# Code Paths
PERIPH_PATH=$(LIBROOT)/STM32F1xx_HAL_Driver
CMSIS_DEVICE=$(LIBROOT)/CMSIS/Device/ST/STM32F1xx
CMSIS_CORE=$(LIBROOT)/CMSIS/Include
DRIVERS=$(PERIPH_PATH)

# Processor specific
LDSCRIPT ?= $(TEMPLATEROOT)/STM32F100CB_FLASH.ld
# STARTUP ?= $(TEMPLATEROOT)/startup/startup_stm32f100xb.s
# SYSTEM ?= $(BUILD)/system_stm32f1xx.o

# Compilation Flags
ARCH_FLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
ARCH_FLAGS += -fsingle-precision-constant -ffast-math
DEF_FLAGS = -DUSE_STDPERIPH_DRIVER -DARM_MATH_CM3 -DSTM32F10X_MD_VL
INC_FLAGS = -I$(CMSIS_CORE) -I$(CMSIS_DEVICE)/Include -I$(PERIPH_PATH)/Inc 
INC_FLAGS += -I$(TEMPLATEROOT)/Source -I$(TEMPLATEROOT)/Inc
CFLAGS += $(ARCH_FLAGS) $(INC_FLAGS) $(DEF_FLAGS)
CFLAGS += -fno-builtin -std=c99
CXXFLAGS += $(ARCH_FLAGS) $(INC_FLAGS) $(DEF_FLAGS)
LDFLAGS += -T$(LDSCRIPT) $(ARCH_FLAGS)

include $(TEMPLATEROOT)/common.mk
