TARGET	:=../bin/autobot
#TOUCH 	 := $(shell touch *)

CC	:= gcc
LINKER   := gcc -o
LCMTYPES := $(wildcard ../lcmtypes/*.lcm)
LCMTYPES := $(LCMTYPES:.lcm=.o)
CFLAGS	:= -c -Wall -g `pkg-config --cflags lcm`
LFLAGS	:= -L../robocape/libraries  -lroboticscape -lm -lrt -lpthread -Wl,-soname,../robocape/libraries/libroboticscape.so `pkg-config --libs lcm`

SOURCES  := $(wildcard *.c)
INCLUDES := $(wildcard *.h)
OBJECTS  := $(SOURCES:$%.c=$%.o)

RM := rm -f

# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(@) $(OBJECTS) $(LFLAGS) $(LCMTYPES)


# compiling command
$(OBJECTS): %.o : %.c
	$(CC) $(CFLAGS) -c $< -o $(@)

all: $(TARGET)
	
clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(TARGET)
