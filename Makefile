# Sprinter Arduino Project Makefile
#
# Makefile Based on:
# Arduino 0011 Makefile
# Arduino adaptation by mellis, eighthave, oli.keller
# Marlin adaption by Daid
#
# This has been tested with Arduino 0022.
#
# This makefile allows you to build sketches from the command line
# without the Arduino environment (or Java).

#Directory used to build files in, contains all the build files.
BUILD_DIR          ?= out

TARGET = $(notdir $(CURDIR))

# VPATH tells make to look into these directory for source files,
# there is no need to specify explicit pathnames as long as the
# directory is added here

VPATH = .
VPATH += $(BUILD_DIR)

CXXSRC = Marlin_main.cpp MarlinSerial.cpp cardreader.cpp \
	planner.cpp stepper.cpp temperature.cpp motion_control.cpp \
	ConfigurationStore.cpp gpio_lib.cpp easyspin.cpp
#CXXSRC = WMath.cpp WString.cpp Print.cpp \
#	Sd2Card.cpp SdBaseFile.cpp SdFatUtil.cpp \
#	SdFile.cpp SdVolume.cpp \
#	watchdog.cpp SPI.cpp Servo.cpp Tone.cpp \
#	vector_3.cpp qr_solve.cpp

# Name of this Makefile (used for "make depend").
MAKEFILE = Makefile

DEBUG = 3

OPT = 0

DEFINES ?=

# Program settings
CC ?= $(CROSS_COMPILE)gcc
CXX ?= $(CROSS_COMPILE)g++
OBJCOPY ?= $(CROSS_COMPILE)objcopy
OBJDUMP ?= $(CROSS_COMPILE)objdump
AR ?= $(CROSS_COMPILE)ar
SIZE ?= $(CROSS_COMPILE)size
NM ?= $(CROSS_COMPILE)nm
REMOVE ?= rm -f
MV ?= mv -f

# Place -D or -U options here
CDEFS    = -fpermissive ${addprefix -D , $(DEFINES)}
CXXDEFS  = $(CDEFS)

# Add all the source directories as include directories too
CINCS = ${addprefix -I ,${VPATH}}
CXXINCS = ${addprefix -I ,${VPATH}}

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
#CSTANDARD = -std=gnu99
CDEBUG = -g$(DEBUG)
CWARN = -Wall
CTUNING =
CEXTRA =

CFLAGS := $(CDEBUG) $(CDEFS) $(CINCS) -O$(OPT) $(CWARN) $(CEXTRA) $(CTUNING)
CXXFLAGS := $(CDEBUG) $(CDEFS) $(CINCS) -O$(OPT) $(CWARN) $(CEXTRA) $(CTUNING)
#ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs
LDFLAGS = -lm -lrt


# Define all object files.
OBJ = ${patsubst %.c, $(BUILD_DIR)/%.o, ${SRC}}
OBJ += ${patsubst %.cpp, $(BUILD_DIR)/%.o, ${CXXSRC}}
OBJ += ${patsubst %.S, $(BUILD_DIR)/%.o, ${ASRC}}

# Define all listing files.
LST = $(ASRC:.S=.lst) $(CXXSRC:.cpp=.lst) $(SRC:.c=.lst)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -I. $(CFLAGS)
ALL_CXXFLAGS = $(CXXFLAGS)
ALL_ASFLAGS = -x assembler-with-cpp $(ASFLAGS)

Pecho=@:
P=

# Default target.
all: sizeafter

build: $(BUILD_DIR) elf lss sym

# Creates the object directory
$(BUILD_DIR):
	$P mkdir -p $(BUILD_DIR)

elf: $(BUILD_DIR)/$(TARGET).elf
lss: $(BUILD_DIR)/$(TARGET).lss
sym: $(BUILD_DIR)/$(TARGET).sym

# Display size of file.
ELFSIZE = $(SIZE)  $(BUILD_DIR)/$(TARGET).elf

sizeafter: build
	$P if [ -f $(BUILD_DIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi

.SUFFIXES: .elf .lss .sym
.PRECIOUS: .o

# Create extended listing file from ELF output file.
.elf.lss:
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
.elf.sym:
	$(NM) -n $< > $@

	# Link: create ELF output file from library.
$(BUILD_DIR)/$(TARGET).elf: $(OBJ) Configuration.h
	$(Pecho) "  CXX   $@"
	$P $(CC) $(ALL_CXXFLAGS) -Wl,--gc-sections -o $@ -L. $(OBJ) $(LDFLAGS)

$(BUILD_DIR)/%.o: %.c Configuration.h Configuration_adv.h $(MAKEFILE)
	$(Pecho) "  CC    $<"
	$P $(CC) -MMD -c $(ALL_CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: $(BUILD_DIR)/%.cpp Configuration.h Configuration_adv.h $(MAKEFILE)
	$(Pecho) "  CXX   $<"
	$P $(CXX) -MMD -c $(ALL_CXXFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Configuration.h Configuration_adv.h $(MAKEFILE)
	$(Pecho) "  CXX   $<"
	$P $(CXX) -MMD -c $(ALL_CXXFLAGS) $< -o $@


# Target: clean project.
clean:
	$(Pecho) "  RM    $(BUILD_DIR)/*"
	$P $(REMOVE) $(BUILD_DIR)/$(TARGET).elf \
		$(BUILD_DIR)/$(TARGET).map $(BUILD_DIR)/$(TARGET).sym $(BUILD_DIR)/$(TARGET).lss $(BUILD_DIR)/$(TARGET).cpp \
		$(OBJ) $(LST) $(SRC:.c=.s) $(SRC:.c=.d) $(CXXSRC:.cpp=.s) $(CXXSRC:.cpp=.d)
	$(Pecho) "  RMDIR $(BUILD_DIR)/"
	$P rm -rf $(BUILD_DIR)


.PHONY:	all build elf lss sym program clean depend sizeafter

# Automaticaly include the dependency files created by gcc
-include ${wildcard $(BUILD_DIR)/*.d}
