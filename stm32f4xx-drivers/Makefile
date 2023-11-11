CC = arm-none-eabi-gcc
MACH = cortex-m4
CFLAGS = -c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall -O0 -Idrivers/Inc
LDFLAGS = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nano.specs -T linker_script.ld -Wl,-Map=blinky.map

./Src/main.o : ./Src/main.c
	$(CC) $(CFLAGS) -o $@ $^

clean :
	rm -rf *.o *.elf ./Src/*.o ./Src/*.elf