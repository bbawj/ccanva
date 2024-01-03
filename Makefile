all:
	cc main.c -Wall -Wextra -lSDL2 -lm -o main && ./main
