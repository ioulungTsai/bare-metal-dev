# Directories
SRC_DIR     := ./Src
STARTUP_DIR := ./Startup
SYSCALLS_DIR:= ./Syscalls
DRIVERS_DIR := ./drivers
OBJ_DIR     := ./Obj
BSP_DIR     := ./bsp

# Compiler and architecture
CC          := arm-none-eabi-gcc
MACH        := cortex-m4

# Compiler options
ARCH_FLAGS  := -mcpu=$(MACH) -mthumb -mfloat-abi=soft
C_STD       := -std=gnu11
C_WARN      := -Wall
C_DEBUG     := -O0 -g3
C_INCLUDES  := -I$(DRIVERS_DIR)/Inc

CFLAGS      := -c $(ARCH_FLAGS) $(C_STD) $(C_WARN) $(C_DEBUG) $(C_INCLUDES)

LDSCRIPT    := -T linker_script.ld
LD_MAP      := -Wl,-Map=$(OBJ_DIR)/stm32f4xx_drivers.map
LD_MAP_SH   := -Wl,-Map=$(OBJ_DIR)/stm32f4xx_drivers_sh.map
LD_SPECS    := --specs=nano.specs
LD_SPECS_SH := --specs=rdimon.specs

LDFLAGS     := $(ARCH_FLAGS) $(LD_SPECS) $(LDSCRIPT) $(LD_MAP)
LDFLAGS_SH  := $(ARCH_FLAGS) $(LD_SPECS_SH) $(LDSCRIPT) $(LD_MAP_SH)

DEPFLAGS    := -MMD -MP

# Source files
FILE        ?= file
SRC         := $(notdir $(FILE))
STARTUP_SRC := $(wildcard $(STARTUP_DIR)/*.c)
DRIVERS_SRC := $(wildcard $(DRIVERS_DIR)/Src/*.c)
SYSCALLS_SRC:= $(wildcard $(SYSCALLS_DIR)/*.c)
BSP_SRC     := $(wildcard $(BSP_DIR)/*.c)

SRCS        := $(STARTUP_SRC) $(DRIVERS_SRC) $(SYSCALLS_SRC) $(BSP_SRC) $(SRC)
SRCS_SH     := $(filter-out $(SYSCALLS_DIR)/syscalls.c, $(SRCS))

# Object files
OBJS        := $(addprefix $(OBJ_DIR)/,$(notdir $(SRCS:.c=.o)))
OBJS_SH     := $(addprefix $(OBJ_DIR)/,$(notdir $(SRCS_SH:.c=.o)))

# Targets
.PHONY: all semi clean load

all: $(OBJ_DIR)/stm32f4xx_drivers.elf

# Run "monitor arm semihosting enable" in GDB to enable the Semihosting feature
semi: $(OBJ_DIR)/stm32f4xx_drivers_sh.elf

# Pattern rules
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) $(DEPFLAGS) -o $@ $<

$(OBJ_DIR)/%.o: $(STARTUP_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) $(DEPFLAGS) -o $@ $<

$(OBJ_DIR)/%.o: $(DRIVERS_DIR)/Src/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) $(DEPFLAGS) -o $@ $<

$(OBJ_DIR)/%.o: $(BSP_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) $(DEPFLAGS) -o $@ $<

$(OBJ_DIR)/%.o: $(SYSCALLS_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) $(DEPFLAGS) -o $@ $<

$(OBJ_DIR)/stm32f4xx_drivers.elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

$(OBJ_DIR)/stm32f4xx_drivers_sh.elf: $(OBJS_SH)
	$(CC) $(LDFLAGS_SH) -o $@ $^

# Ensure the object directory exists
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

clean:
	rm -rf $(OBJ_DIR)

load:
	openocd -f board/stm32f4discovery.cfg

# Include dependency files
-include $(OBJ_DIR)/*.d
