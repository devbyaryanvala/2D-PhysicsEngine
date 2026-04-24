# Makefile for 2D Physics Engine
CC = gcc

# Adjust these paths based on your SDL2 installation
CFLAGS = -Iinclude -IC:/SDL2/i686-w64-mingw32/include -Wall -std=c99
LDFLAGS = -LC:/SDL2/i686-w64-mingw32/lib -lmingw32 -lSDL2main -lSDL2 -lm

# Source files and objects
SRC = $(wildcard src/*.c)
OBJ = $(patsubst src/%.c, build/obj/%.o, $(SRC))

# Demo files and executables
DEMOS = $(wildcard demos/*.c)
DEMO_BINS = $(patsubst demos/%.c, build/%.exe, $(DEMOS))

# Default target
all: prepare $(DEMO_BINS)

# Compile engine source into object files
build/obj/%.o: src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Compile demos and link with engine objects
build/%.exe: demos/%.c $(OBJ)
	$(CC) $(CFLAGS) $< $(OBJ) -o $@ $(LDFLAGS)

# Create build directories
prepare:
	@if not exist build mkdir build
	@if not exist build\obj mkdir build\obj
	
# Clean target
clean:
	@if exist build rmdir /s /q build

# Run a specific demo (e.g., make run DEMO=demo6_ccd)
run: all
ifndef DEMO
	@echo Please specify a demo to run. Example: make run DEMO=demo6_ccd
else
	.\build\$(DEMO).exe
endif