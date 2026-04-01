CC      = gcc
CFLAGS  = $(shell pkg-config --cflags gtk+-3.0) -Wall -Wextra -O2
LIBS    = $(shell pkg-config --libs   gtk+-3.0)

TARGET  = cave_ui
SRC     = cave_ui.c

.PHONY: all clean install-deps

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)

# Install GTK3 development headers (run once)
install-deps:
	sudo apt-get install -y libgtk-3-dev

clean:
	rm -f $(TARGET)
