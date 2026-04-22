CC = gcc

CFLAGS = -Iinclude -IC:/SDL2/i686-w64-mingw32/include -Wall -std=c99

SRC = $(wildcard src/*.c)
OUT = build/engine.exe

all:
	$(CC) $(SRC) $(CFLAGS) -o $(OUT) \
	-LC:/SDL2/i686-w64-mingw32/lib \
	-lmingw32 -lSDL2main -lSDL2 -lm

run: all
	./build/engine.exe