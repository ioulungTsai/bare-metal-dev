CC = arm-none-eabi-gcc
MACH = cortex-m4
CFLAGS = -c -mcpu=$(MACH) -mthumb -std=gnu11 -Wall -O0

all : main.o led.o startup.o
 
main.o : main.c
	$(CC) $(CFLAGS) -o $@ $^

led.o : led.c
	$(CC) $(CFLAGS) -o $@ $^

startup.o : startup.c
	$(CC) $(CFLAGS) -o $@ $^

clean :
	rm -rf *.o *.elf
	