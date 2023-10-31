CC = arm-none-eabi-gcc
MACH = cortex-m4
CFLAGS = -c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall -O0
LDFLAGS = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nano.specs -T linker_script.ld -Wl,-Map=blinky.map

all : main.o led.o startup.o syscalls.o blinky.elf
 
main.o : main.c
	$(CC) $(CFLAGS) -o $@ $^

led.o : led.c
	$(CC) $(CFLAGS) -o $@ $^

startup.o : startup.c
	$(CC) $(CFLAGS) -o $@ $^

syscalls.o : syscalls.c
	$(CC) $(CFLAGS) -o $@ $^

blinky.elf : main.o led.o startup.o syscalls.o
	$(CC) $(LDFLAGS) -o $@ $^

clean :
	rm -rf *.o *.elf

load : 
	openocd -f board/stm32f4discovery.cfg
