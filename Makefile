# Makefile for STM32F4
# 11-10-2011 E. Brombaugh

# sub directories
VPATH = .:CMSIS:StdPeriph

# Object files
OBJECTS = 	startup_stm32f4xx.o
OBJECTS +=	system_stm32f4xx.o
OBJECTS +=	main.o

OBJECTS +=	stm32f4xx_dac.o
OBJECTS +=	stm32f4xx_gpio.o
OBJECTS +=	stm32f4xx_i2c.o
OBJECTS +=	stm32f4xx_rcc.o
OBJECTS +=	stm32f4xx_spi.o
OBJECTS +=	stm32f4xx_dma.o
OBJECTS +=	stm32f4xx_adc.o
OBJECTS +=  misc.o
OBJECTS +=	stm32f4xx_usart.o

OBJECTS +=  except.o

# Linker script
LDSCRIPT = stm32f407.ld

CFLAGS = -g -O0 -mlittle-endian -mthumb -ffunction-sections 
CFLAGS += -I. -ICMSIS -IStdPeriph -DARM_MATH_CM4 -D'__FPU_PRESENT=1'
CFLAGS += -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 
# CFLAGS += -mcpu=cortex-m4
AFLAGS  = -mlittle-endian -mthumb -mcpu=cortex-m4 
LFLAGS  = $(CFLAGS) -nostartfiles -T $(LDSCRIPT) -Wl,-Map=main.map
#LFLAGS += -Wl,--gc-sections --specs=nano.specs
LFLAGS += -Wl,--gc-sections

# Executables
ARCH = arm-none-eabi
CC = $(ARCH)-gcc
LD = $(ARCH)-ld -v
AS = $(ARCH)-as
OBJCPY = $(ARCH)-objcopy
OBJDMP = $(ARCH)-objdump
GDB = $(ARCH)-gdb

#CPFLAGS = --output-target=binary -j .text -j .data
CPFLAGS = --output-target=binary
ODFLAGS	= -x --syms

FLASH = st-flash

# Targets
all: main.bin

clean:
	-rm -f $(OBJECTS) *.lst *.elf *.bin *.map *.dmp

flash: gdb_flash
	
gdb_flash: main.elf
	$(GDB) -x flash_cmd.gdb -batch

debug: main.elf
	ddd --debugger $(GDB) main.elf

disassemble: main.elf
	$(OBJDMP) -dS main.elf > main.dis

main.ihex: main.elf
	$(OBJCPY) --output-target=ihex main.elf main.ihex

main.bin: main.elf 
	$(OBJCPY) $(CPFLAGS) main.elf main.bin
	$(OBJDMP) $(ODFLAGS) main.elf > main.dmp
	ls -l main.elf main.bin

main.elf: $(OBJECTS) $(LDSCRIPT)
	$(CC) $(LFLAGS) -o main.elf $(OBJECTS) -lnosys -lm

%.o: %.c %.h
	$(CC) $(CFLAGS) -c -o $@ $<

