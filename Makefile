all:
	cc main.c -Wall -Wextra -ggdb -lSDL2 -lm -o main && ./main
