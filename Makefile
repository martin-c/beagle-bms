INCLUDE_DIRS = 
LIB_DIRS = 
CC = gcc

# compiler flags
CDEFS =
CFLAGS += -O3
CFLAGS += -Wall
CFLAGS += -std=gnu99
CFLAGS += $(INCLUDE_DIRS)

# libraries
LIBS := -pthread -lrt -lncurses

# build target name
TARGET := beagle_bms
SOURCES := $(TARGET).c
SOURCES += beagle_spi.c
SOURCES += cell.c
SOURCES += ltc6804_util.c
SOURCES += ltc_6804.c
SOURCES +=


# objects
OBJS := $(SOURCES:%.c=%.o)

# rules
all:	$(TARGET)

clean:
	-rm -f *.o *.NEW *~
	-rm -f $(TARGET) $(DERIVED) $(GARBAGE)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LIBS)

.c.o:
	$(CC) -c $(CFLAGS) -o $@ $<


	
