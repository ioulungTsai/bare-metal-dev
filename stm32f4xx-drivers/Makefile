SRC_DIR 		=	./Src
STARTUP_DIR 	=	./Startup
SYSCALLS_DIR	=	./Syscalls
DRIVERS_DIR 	=	./drivers
OBJ_DIR 		=	./Obj

CC 				= 	arm-none-eabi-gcc
MACH 			= 	cortex-m4
CFLAGS 			= 	-c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall -O0 -g3 -I$(DRIVERS_DIR)/Inc
LDFLAGS			= 	-mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nano.specs -T linker_script.ld -Wl,-Map=./$(OBJ_DIR)/stm32f4xx_drivers.map
DEPFLAGS 		= 	-MMD -MP -MF $*.d

FILE			=   file
SRC				=	$(notdir $(FILE))
OBJ				=	$(SRC:.c=.o)
SRCS 			= 	$(shell find $(STARTUP_DIR) $(DRIVERS_DIR) $(SYSCALLS_DIR) -name *.c) $(SRC)
OBJS 			=   $(addprefix $(OBJ_DIR)/,$(notdir $(SRCS:.c=.o)))


.PHONY: all clean load

all: ./$(OBJ_DIR)/stm32f4xx_drivers.elf

./$(OBJ_DIR)/$(OBJ): $(SRC_DIR)/$(SRC)
	$(CC) $(CFLAGS) $(DEPFLAGS) -o $@ $^

./$(OBJ_DIR)/startup.o: $(STARTUP_DIR)/startup.c
	$(CC) $(CFLAGS) $(DEPFLAGS) -o $@ $^

./$(OBJ_DIR)/%.o: $(DRIVERS_DIR)/Src/%.c
	$(CC) $(CFLAGS) $(DEPFLAGS) -o $@ $^

./$(OBJ_DIR)/syscalls.o: $(SYSCALLS_DIR)/syscalls.c
	$(CC) $(CFLAGS) $(DEPFLAGS) -o $@ $^

./$(OBJ_DIR)/stm32f4xx_drivers.elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

clean:
	rm -rf *.o *.elf *.map $(OBJ_DIR)
	mkdir -p $(OBJ_DIR)

load : 
	openocd -f board/stm32f4discovery.cfg

-include $(OBJ_DIR)/*.d
