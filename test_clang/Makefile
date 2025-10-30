# ==== toolchain paths ====
LLVM_BIN := /Users/xiaolonghuang/Documents/LLVM_ARM/arm-toolchain/arm-software/embedded/build/llvm/bin
CLANG    := $(LLVM_BIN)/clang
OBJCOPY  := $(LLVM_BIN)/llvm-objcopy
SIZE     := $(LLVM_BIN)/llvm-size

# Set this to your installed picolibc/newlib sysroot (must contain include/, lib/, crt*.o)
SYSROOT  := /path/to/picolibc/install/arm-none-eabi

# ==== target / MCU ====
TRIPLE   := thumbv7em-none-eabihf
MCUFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# ==== compile flags ====
CFLAGS   := -Os -ffreestanding -fno-exceptions -fno-rtti $(MCUFLAGS)
CPPFLAGS := --target=$(TRIPLE) --sysroot=$(SYSROOT)

# ==== (optional) link flags if/when you’re ready to link ====
# Provide your linker script path (e.g., stm32g4.ld)
LD_SCRIPT ?= stm32g4.ld
LDFLAGS   := -fuse-ld=lld -nostdlib -Wl,--gc-sections -Wl,-Map=build/firmware.map \
             $(MCUFLAGS) -rtlib=compiler-rt

# ==== project files ====
SRCS   := main.c
OBJDIR := build
OBJS   := $(SRCS:%.c=$(OBJDIR)/%.o)

# Final artifacts (optional, if you link)
ELF := $(OBJDIR)/firmware.elf
HEX := $(OBJDIR)/firmware.hex
BIN := $(OBJDIR)/firmware.bin

# ==== default ====
.PHONY: all clean size
all: $(OBJS)
# To build a full firmware, run:  make firmware

# ==== compile ====
$(OBJDIR):
	@mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: %.c | $(OBJDIR)
	$(CLANG) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# ==== (optional) link + artifacts ====
.PHONY: firmware
firmware: $(ELF) $(HEX) $(BIN) size

$(ELF): $(OBJS) $(LD_SCRIPT)
	$(CLANG) $(CPPFLAGS) $(LDFLAGS) -T $(LD_SCRIPT) $(OBJS) -lc -lm -lnosys -o $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

size: $(ELF)
	$(SIZE) --format=SysV $(ELF)

# ==== clean ====
clean:
	rm -rf $(OBJDIR)
