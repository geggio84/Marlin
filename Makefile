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
#
# Detailed instructions for using the makefile:
#
#  1. Modify the line containg "ARDUINO_INSTALL_DIR" to point to the directory that
#     contains the Arduino installation (for example, under Mac OS X, this
#     might be /Applications/Arduino.app/Contents/Resources/Java).
#
#  2. Modify the line containing "UPLOAD_PORT" to refer to the filename
#     representing the USB or serial connection to your Arduino board
#     (e.g. UPLOAD_PORT = /dev/tty.USB0).  If the exact name of this file
#     changes, you can use * as a wildcard (e.g. UPLOAD_PORT = /dev/tty.usb*).
#
#  3. Set the line containing "MCU" to match your board's processor.
#     Older one's are atmega8 based, newer ones like Arduino Mini, Bluetooth
#     or Diecimila have the atmega168.  If you're using a LilyPad Arduino,
#     change F_CPU to 8000000. If you are using Gen7 electronics, you
#     probably need to use 20000000. Either way, you must regenerate
#     the speed lookup table with create_speed_lookuptable.py.
#
#  4. Type "make" and press enter to compile/verify your program.
#
#  5. Type "make upload", reset your Arduino board, and press enter to
#     upload your program to the Arduino board.
#
# Note that all settings are set with ?=, this means you can override them
# from the commandline with "make HARDWARE_MOTHERBOARD=71" for example

# This defined the board you are compiling for (see Configuration.h for the options)
HARDWARE_MOTHERBOARD ?= 999

#Directory used to build files in, contains all the build files, from object files to the final hex file.
BUILD_DIR          ?= out

TARGET = $(notdir $(CURDIR))

# VPATH tells make to look into these directory for source files,
# there is no need to specify explicit pathnames as long as the
# directory is added here

VPATH = .
VPATH += $(BUILD_DIR)

CXXSRC = Marlin_main.cpp MarlinSerial.cpp cardreader.cpp \
	planner.cpp stepper.cpp temperature.cpp motion_control.cpp \
	ConfigurationStore.cpp
#CXXSRC = WMath.cpp WString.cpp Print.cpp Marlin_main.cpp	\
	MarlinSerial.cpp Sd2Card.cpp SdBaseFile.cpp SdFatUtil.cpp	\
	SdFile.cpp SdVolume.cpp motion_control.cpp planner.cpp		\
	stepper.cpp temperature.cpp cardreader.cpp ConfigurationStore.cpp \
	watchdog.cpp SPI.cpp Servo.cpp Tone.cpp ultralcd.cpp digipot_mcp4451.cpp \
	vector_3.cpp qr_solve.cpp

# Name of this Makefile (used for "make depend").
MAKEFILE = Makefile

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
DEBUG = 3

OPT = 0

DEFINES ?=

# Program settings
CC = gcc
CXX = g++
OBJCOPY = objcopy
OBJDUMP = objdump
AR  = ar
SIZE = size
NM = nm
REMOVE = rm -f
MV = mv -f

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
CWARN = -Wall -Wstrict-prototypes
CTUNING =
CEXTRA =

CFLAGS := $(CDEBUG) $(CDEFS) $(CINCS) -O$(OPT) $(CWARN) $(CEXTRA) $(CTUNING)
CXXFLAGS := $(CDEBUG) $(CDEFS) $(CINCS) -O$(OPT) -Wall    $(CEXTRA) $(CTUNING)
#ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs
LDFLAGS = -lm


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

build: $(BUILD_DIR) elf

# Creates the object directory
$(BUILD_DIR):
	$P mkdir -p $(BUILD_DIR)

elf: $(BUILD_DIR)/$(TARGET).elf
hex: $(BUILD_DIR)/$(TARGET).hex
eep: $(BUILD_DIR)/$(TARGET).eep
lss: $(BUILD_DIR)/$(TARGET).lss
sym: $(BUILD_DIR)/$(TARGET).sym

	# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(BUILD_DIR)/$(TARGET).hex
ELFSIZE = $(SIZE)  $(BUILD_DIR)/$(TARGET).elf

sizeafter: build
	$P if [ -f $(BUILD_DIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi


# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
	--change-section-address .data-0x800000 \
	--change-section-address .bss-0x800000 \
	--change-section-address .noinit-0x800000 \
	--change-section-address .eeprom-0x810000


coff: $(BUILD_DIR)/$(TARGET).elf
	$(COFFCONVERT) -O coff-avr $(BUILD_DIR)/$(TARGET).elf $(TARGET).cof


extcoff: $(TARGET).elf
	$(COFFCONVERT) -O coff-ext-avr $(BUILD_DIR)/$(TARGET).elf $(TARGET).cof


.SUFFIXES: .elf .hex .eep .lss .sym
.PRECIOUS: .o

.elf.hex:
	$(Pecho) "  COPY  $@"
	$P $(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

.elf.eep:
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
		--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

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
	$P $(REMOVE) $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).eep $(BUILD_DIR)/$(TARGET).cof $(BUILD_DIR)/$(TARGET).elf \
		$(BUILD_DIR)/$(TARGET).map $(BUILD_DIR)/$(TARGET).sym $(BUILD_DIR)/$(TARGET).lss $(BUILD_DIR)/$(TARGET).cpp \
		$(OBJ) $(LST) $(SRC:.c=.s) $(SRC:.c=.d) $(CXXSRC:.cpp=.s) $(CXXSRC:.cpp=.d)
	$(Pecho) "  RMDIR $(BUILD_DIR)/"
	$P rm -rf $(BUILD_DIR)


.PHONY:	all build elf hex eep lss sym program coff extcoff clean depend sizeafter

# Automaticaly include the dependency files created by gcc
-include ${wildcard $(BUILD_DIR)/*.d}
