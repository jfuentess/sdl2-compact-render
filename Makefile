CC=g++
SDSL-LITE-FLDR=/home/jose
CFLAGS=-O3

render: main.cpp SimplexNoise.cpp
	$(CC) $(CFLAGS) -o render main.cpp SimplexNoise.cpp `sdl2-config --cflags --libs` -I./k3-lidar-lite -I$(SDSL-LITE-FLDR)/include  -L$(SDSL-LITE-FLDR)/lib -lsdsl

