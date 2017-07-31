LCM = lcm-gen
LCMTYPES_NAMES := $(shell ls *.lcm)

# Construct the pathnames to the C types
LCMTYPES_C := $(LCMTYPES_NAMES:%.lcm=%.c)
LCMTYPES_O = $(LCMTYPES_C:%.c=%.o)
CC = gcc
CFLAGS = -g `pkg-config --cflags lcm`

# Construct pathnames to python types
PY_PACKAGE = lcmtypes
LCMTYPES_PY := $(LCMTYPES_NAMES:%.lcm=%.py)

# Java build options
BUILD_PATH = build
LCMTYPES_JAVA := $(LCMTYPES_NAMES:%.lcm=%.java)
LCMTYPES_JAVA := $(addprefix $(BUILD_PATH)/lcmtypes/,$(LCMTYPES_JAVA))
LCM_JAR = /usr/local/share/java/lcm.jar

ALL = $(LCMTYPES_C) $(LCMTYPES_O) $(LCMTYPES_PY)
ifneq "$(shell which javac)" ""
ALL += lcmtypes.jar
endif

all: $(ALL)

%.c: %.lcm
	$(LCM) -c --c-typeinfo $<

%.o: %.c
	$(CC) $(CFLAGS) -c $^ -o $@

%.py: %.lcm
	$(LCM) -p --package-prefix $(PY_PACKAGE) $<

clean:
	rm -rf build/
	rm -rf $(PY_PACKAGE)/
	rm -f *.c *.h *.o *.py *~ lcmtypes.jar

$(BUILD_PATH)/lcmtypes/%.java: %.lcm
	$(LCM) -j --jpath=$(BUILD_PATH) $<

lcmtypes.jar: $(LCMTYPES_JAVA)
	mkdir -p $(BUILD_PATH)/class
	javac -cp $(LCM_JAR) -d $(BUILD_PATH)/class $^
	jar cf $@ -C $(BUILD_PATH)/class .


