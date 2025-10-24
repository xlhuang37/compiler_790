TOOL=arm-none-eabi-
CC=$(TOOL)gcc
OBJCOPY=$(TOOL)objcopy
SIZE=$(TOOL)size

MCUFLAGS=-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS=$(MCUFLAGS) -Os -ffunction-sections -fdata-sections -Wall -Wextra -g
LDFLAGS=-T stm32g4.ld -Wl,--gc-sections -nostartfiles -Wl,-Map=firmware.map
TARGET=firmware

all: $(TARGET).bin

$(TARGET).elf: startup.o main.o stm32g4.ld
	$(CC) $(CFLAGS) startup.o main.o -o $@ $(LDFLAGS)
	$(SIZE) $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f *.o *.elf *.bin *.map

