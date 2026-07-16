# Motor-MCU bootloader build. Self-contained — invoke from the project root:
#   make -C boot
# Produces boot/build/boot.elf/.bin/.hex/.map. Flash boot.bin at 0x08000000.
#
# Uses the same toolchain as the CubeIDE app build; override TOOLCHAIN_BIN on
# the command line if your install layout differs:
#   make -C boot TOOLCHAIN_BIN=/path/to/arm-none-eabi/bin

TARGET    := boot
BUILD_DIR := build

TOOLCHAIN_BIN ?= C:/ST/STM32CubeIDE_1.12.1/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.100.202509120712/tools/bin
CC      := $(TOOLCHAIN_BIN)/arm-none-eabi-gcc
OBJCOPY := $(TOOLCHAIN_BIN)/arm-none-eabi-objcopy
SIZE    := $(TOOLCHAIN_BIN)/arm-none-eabi-size

# Project root (one level up from boot/) and the shared contract folder.
ROOT  := ..
IFACE := ../../Generic_axis_controller/Generic_axis_controller/Interface

# -- source lists ----------------------------------------------------------
# Bootloader-local sources. No net/log/identity deps (motor talks SPI).
BOOT_SRCS := \
  boot_main.c \
  boot_flag.c \
  boot_od.c \
  boot_seg_sdo.c \
  boot_flash.c \
  boot_spi.c \
  boot_stubs.c

# Cube-generated files reused from the app tree. spi.c and stm32g4xx_it.c are
# DELIBERATELY excluded — boot_spi.c provides its own MX_SPI2_Init (no DMA) and
# boot_stubs.c provides the minimal interrupt handlers (Cube's it.c references
# app-only handles: hadc*, htim*, hdma_*). system_stm32g4xx.c is compiled with
# VECT_TAB_OFFSET=0U so the bootloader's VTOR stays at 0x08000000.
CORE_SRCS := \
  $(ROOT)/Core/Src/system_stm32g4xx.c \
  $(ROOT)/Core/Src/gpio.c \
  $(ROOT)/Core/Src/stm32g4xx_hal_msp.c

HAL_DIR  := $(ROOT)/Drivers/STM32G4xx_HAL_Driver/Src
HAL_SRCS := \
  $(HAL_DIR)/stm32g4xx_hal.c \
  $(HAL_DIR)/stm32g4xx_hal_cortex.c \
  $(HAL_DIR)/stm32g4xx_hal_dma.c \
  $(HAL_DIR)/stm32g4xx_hal_flash.c \
  $(HAL_DIR)/stm32g4xx_hal_flash_ex.c \
  $(HAL_DIR)/stm32g4xx_hal_flash_ramfunc.c \
  $(HAL_DIR)/stm32g4xx_hal_gpio.c \
  $(HAL_DIR)/stm32g4xx_hal_pwr.c \
  $(HAL_DIR)/stm32g4xx_hal_pwr_ex.c \
  $(HAL_DIR)/stm32g4xx_hal_rcc.c \
  $(HAL_DIR)/stm32g4xx_hal_rcc_ex.c \
  $(HAL_DIR)/stm32g4xx_hal_spi.c \
  $(HAL_DIR)/stm32g4xx_hal_spi_ex.c \
  $(HAL_DIR)/stm32g4xx_hal_exti.c

# Adjust if your startup file name differs.
ASM_SRCS := $(ROOT)/Core/Startup/startup_stm32g474retx.s

# -- output objects --------------------------------------------------------
C_OBJS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(notdir $(BOOT_SRCS))) \
          $(patsubst $(ROOT)/%,$(BUILD_DIR)/%.o,$(basename $(CORE_SRCS) $(HAL_SRCS)))
ASM_OBJS := $(patsubst $(ROOT)/%,$(BUILD_DIR)/%.o,$(basename $(ASM_SRCS)))
ALL_OBJS := $(C_OBJS) $(ASM_OBJS)

# -- flags -----------------------------------------------------------------
INCLUDES := \
  -I$(ROOT)/Core/Inc \
  -I$(ROOT)/Drivers/STM32G4xx_HAL_Driver/Inc \
  -I$(ROOT)/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy \
  -I$(ROOT)/Drivers/CMSIS/Device/ST/STM32G4xx/Include \
  -I$(ROOT)/Drivers/CMSIS/Include \
  -I$(IFACE) \
  -I.

MCU_FLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS      := -DSTM32G474xx -DUSE_HAL_DRIVER
CFLAGS    := $(MCU_FLAGS) $(DEFS) $(INCLUDES) \
             -std=gnu11 -g3 -O0 \
             -ffunction-sections -fdata-sections \
             -Wall --specs=nano.specs
# Keep the bootloader VTOR at 0x08000000. USER_VECT_TAB_ADDRESS makes
# system_stm32g4xx.c's SystemInit set SCB->VTOR = FLASH_BASE | 0 = 0x08000000
# (its default VECT_TAB_OFFSET is 0). The app instead sets VTOR in main().
CFLAGS_BOOT_ONLY := -DUSER_VECT_TAB_ADDRESS

ASFLAGS := $(MCU_FLAGS) -g3 -x assembler-with-cpp $(DEFS)

LDFLAGS := $(MCU_FLAGS) -T$(CURDIR)/boot_link.ld \
           --specs=nano.specs \
           -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
           -Wl,--gc-sections \
           -static -Wl,--start-group -lc -lm -Wl,--end-group

# -- rules -----------------------------------------------------------------
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex
	@$(SIZE) $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR)/boot_%.o: boot_%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CFLAGS_BOOT_ONLY) -c $< -o $@

$(BUILD_DIR)/%.o: $(ROOT)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CFLAGS_BOOT_ONLY) -c $< -o $@

$(BUILD_DIR)/%.o: $(ROOT)/%.s
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(ALL_OBJS) boot_link.ld | $(BUILD_DIR)
	$(CC) $(ALL_OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean
