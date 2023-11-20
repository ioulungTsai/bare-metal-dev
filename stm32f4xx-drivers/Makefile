CC = arm-none-eabi-gcc
MACH = cortex-m4
CFLAGS = -c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall -O0 -Idrivers/Inc
LDFLAGS = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nano.specs -T linker_script.ld -Wl,-Map=stm32f4xx_drivers.map


all : ./Src/main.o ./Startup/startup.o ./Syscalls/syscalls.o ./Src/stm32f4xx_drivers.elf

./Src/main.o : ./Src/main.c
	$(CC) $(CFLAGS) -o $@ $^

./Startup/startup.o : ./Startup/startup.c
	$(CC) $(CFLAGS) -o $@ $^

./Syscalls/syscalls.o : ./Syscalls/syscalls.c
	$(CC) $(CFLAGS) -o $@ $^

./Src/stm32f4xx_drivers.elf : ./Src/main.o ./Startup/startup.o ./Syscalls/syscalls.o
	$(CC) $(LDFLAGS) -o $@ $^

clean :
	rm -rf *.o *.elf ./Src/*.o ./Startup/*.o ./Syscalls/*.o ./Src/*.elf
