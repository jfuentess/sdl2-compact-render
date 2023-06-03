all:
	g++ -IC:\Dev\i686-w64-mingw32\include\SDL2 -IC:\Dev\glm -LC:\Dev\i686-w64-mingw32\lib -o main main.cpp SimplexNoise.cpp -lmingw32 -lSDL2main -lSDL2 -std=c++11 -static-libgcc