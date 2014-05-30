###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
# 

###############################################################################
# Things that the user might override on the commandline
#

# The target to build
TARGET		?=  NAZEPRO

# The name of the build
BRANCH_NAME     = dev

# Compile-time options
OPTIONS		?=

# Debugger options, must be empty or GDB
DEBUG ?=

# Serial port/Device for flashing
SERIAL_DEVICE	?= /dev/ttyUSB0


###############################################################################
# Things that need to be maintained as the source changes
#

VALID_TARGETS	 = NAZEPRO NAZE


# Common working directories
ROOT		 = $(dir $(lastword $(MAKEFILE_LIST)))
SRC_DIR		 = $(ROOT)/src
OBJECT_DIR	 = $(ROOT)/obj
BIN_DIR		 = $(ROOT)/obj
CMSIS_DIR	 = $(ROOT)/lib/CMSIS
INCLUDE_DIRS = $(SRC_DIR)

# Common Search path for sources
VPATH		:= $(SRC_DIR):$(SRC_DIR)/startup




# STM32F303xC 
ifeq ($(TARGET),$(filter $(TARGET), NAZEPRO))

DIR_STDPERIPH = $(ROOT)/lib/STM32F30x_StdPeriph_Driver
DIR_USBFS     = $(ROOT)/lib/STM32_USB-FS-Device_Driver

CMSIS_SRC	  = $(notdir $(wildcard $(CMSIS_DIR)/Device/ST/STM32F30x/Source/Templates/*.c))

VPATH		:= $(VPATH):$(CMSIS_DIR)/Device/ST/STM32F30x/Source/Templates/

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(DIR_STDPERIPH)/inc \
		   $(DIR_USBFS)/inc \
		   $(CMSIS_DIR)/Include \
		   $(CMSIS_DIR)/Device/ST/STM32F30x/Include \
		   $(ROOT)/src/vcp


LD_SCRIPT	 = $(ROOT)/stm32_flash_f303.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 

DEVICE_FLAGS = -DSTM32F303xC -DARM_MATH_CM4 
DEVICE_MCUNAME = stm32f30x

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(DIR_STDPERIPH)/src:$(DIR_USBFS)/src

USBPERIPH_SRC = $(notdir $(wildcard $(DIR_USBFS)/src/*.c))
STDPERIPH_SRC = $(notdir $(wildcard $(DIR_STDPERIPH)/src/*.c))

DEVICE_STDPERIPH_SRC = $(USBPERIPH_SRC) \
$(STDPERIPH_SRC)

endif
#end STM32F303xC





# STM32F10x 
ifeq ($(TARGET),$(filter $(TARGET), NAZE))

DIR_STDPERIPH = $(ROOT)/lib/STM32F10x_StdPeriph_Driver

CMSIS_SRC	  = $(notdir $(wildcard $(CMSIS_DIR)/Device/ST/STM32F10x/Source/Templates/*.c))

VPATH		:= $(VPATH):$(CMSIS_DIR)/Device/ST/STM32F10x/Source/Templates/

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(DIR_STDPERIPH)/inc \
		   $(CMSIS_DIR)/Include \
		   $(CMSIS_DIR)/Device/ST/STM32F10x/Include


LD_SCRIPT	 = $(ROOT)/stm32_flash_f103.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3

DEVICE_FLAGS = -DSTM32F10X_MD 
DEVICE_MCUNAME = stm32f10x

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(DIR_STDPERIPH)/src

STDPERIPH_SRC = $(notdir $(wildcard $(DIR_STDPERIPH)/src/*.c))

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)


endif
#end STM32F10x


MW_SRC	 = \
 		align.c \
		buzzer.c \
		cli.c \
		config.c \
		gps.c \
		imu.c \
		main.c \
		mw.c \
		mixer.c \
		msp.c \
		printf.c \
		rxmsp.c \
		rxsbus.c \
		rxspektrum.c \
		rxsumd.c \
		sensors.c \
		telemetry_common.c \
		telemetry_frsky.c \
		telemetry_hott.c \
		utils_math.c \
		$(CMSIS_SRC) \
		$(DEVICE_STDPERIPH_SRC) \
		startup/startup_$(DEVICE_MCUNAME)_gcc.s 

COMMON_SRC =  \
		drv/adc_$(DEVICE_MCUNAME).c \
		drv/gpio_$(DEVICE_MCUNAME).c \
		drv/spi_$(DEVICE_MCUNAME).c \
		drv/i2c_$(DEVICE_MCUNAME).c \
		drv/board_$(DEVICE_MCUNAME).c \
		drv/uart_$(DEVICE_MCUNAME).c \

NAZEPRO_SRC = $(MW_SRC) \
		$(COMMON_SRC) \
		drv/usb.c \
		drv/crc.c \
		drv/i2c_soft.c \
		drv/pwm.c \
		drv/serial.c \
		drv/softserial.c \
		drv/timer.c \
		drv/uart.c \
		sensors/i2c_adxl345.c \
		sensors/i2c_bma280.c \
		sensors/i2c_bmp085.c \
		sensors/i2c_hmc5883l.c \
		sensors/i2c_l3g4200d.c \
		sensors/i2c_ledring.c \
		sensors/i2c_mma845x.c \
		sensors/i2c_mpu3050.c \
		sensors/i2c_mpu6050.c \
		sensors/i2c_ms5611.c \
		sensors/spi_hmc5983.c \
		sensors/spi_mpu6000.c \
		sensors/spi_ms5611.c \
		vcp/hw_config.c \
		vcp/stm32_it.c \
		vcp/usb_desc.c \
		vcp/usb_endp.c \
		vcp/usb_istr.c \
		vcp/usb_prop.c \
		vcp/usb_pwr.c

NAZE_SRC = $(MW_SRC) \
		$(COMMON_SRC) \
		drv/i2c_soft \
		drv/pwm.c \
		drv/serial.c \
		drv/softserial.c \
		drv/timer.c \
		drv/uart.c \
		sensors/i2c_adxl345.c \
		sensors/i2c_bma280.c \
		sensors/i2c_bmp085.c \
		sensors/i2c_hmc5883l.c \
		sensors/i2c_l3g4200d.c \
		sensors/i2c_ledring.c \
		sensors/i2c_mma845x.c \
		sensors/i2c_mpu3050.c \
		sensors/i2c_mpu6050.c \
		sensors/i2c_ms5611.c 
		
#				drv/uart_f303.c \
		
# In some cases, %.s regarded as intermediate file, which is actually not.
# This will prevent accidental deletion of startup code.
.PRECIOUS: %.s


###############################################################################
# Things that might need changing to use different tools
#

# Tool names
CC		= arm-none-eabi-gcc
OBJCOPY	= arm-none-eabi-objcopy

#
# Tool options.
BASE_CFLAGS	 = $(ARCH_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   -Wall \
		   -ffunction-sections \
		   -fdata-sections \
		   $(DEVICE_FLAGS) \
		   -DUSE_STDPERIPH_DRIVER \
		   -D$(TARGET) \
		   -D'__BRANCH_NAME__="$(BRANCH_NAME)"' 

ASFLAGS		 = $(ARCH_FLAGS) \
		   -x assembler-with-cpp \
		   $(addprefix -I,$(INCLUDE_DIRS))

# XXX Map/crossref output?
LDFLAGS		 = -lm \
		   $(ARCH_FLAGS) \
		   -static \
		   -Wl,-gc-sections,-Map,$(TARGET_MAP) \
		   -T$(LD_SCRIPT)



###############################################################################
# No user-serviceable parts below
###############################################################################
#
# Things we will build
#
ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS))
endif

ifeq ($(DEBUG),GDB)
CFLAGS = $(BASE_CFLAGS) \
	-ggdb \
	-O0
else
CFLAGS = $(BASE_CFLAGS) \
	-Os
endif


TARGET_HEX	 = $(BIN_DIR)/$(BRANCH_NAME)_$(TARGET).hex
TARGET_ELF	 = $(BIN_DIR)/$(BRANCH_NAME)_$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_MAP   = $(OBJECT_DIR)/$(BRANCH_NAME)_$(TARGET).map

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 
$(OBJECT_DIR)/$(TARGET)/%.o): %.S
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 

clean:
	rm -f $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
	rm -rf $(OBJECT_DIR)/$(TARGET)

flash_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	echo -n 'R' >$(SERIAL_DEVICE)
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

flash: flash_$(TARGET)


unbrick_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

unbrick: unbrick_$(TARGET)

help:
	@echo ""
	@echo "Makefile for the $(BRANCH_NAME)_$(TARGET) firmware"
	@echo ""
	@echo "Usage:"
	@echo "        make [TARGET=<target>] [OPTIONS=\"<options>\"]"
	@echo ""
	@echo "Valid TARGET values are: $(VALID_TARGETS)"
	@echo ""
