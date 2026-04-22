# Makefile for 2D Physics Engine
CC = gcc

# Adjust these paths based on your SDL2 installation
CFLAGS = -Iinclude -IC:/SDL2/i686-w64-mingw32/include -Wall -std=c99
LDFLAGS = -LC:/SDL2/i686-w64-mingw32/lib -lmingw32 -lSDL2main -lSDL2 -lm

# Source files and output executable
SRC = $(wildcard src/*.c)
OUT = build/physics_engine.exe

# Default target
all: prepare
	$(CC) $(SRC) $(CFLAGS) -o $(OUT) $(LDFLAGS)

# Create build directory if it doesn't exist
prepare:
	@if not exist build mkdir build
	
# Clean target to remove build artifacts
run: all
	./$(OUT)