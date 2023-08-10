#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>

using namespace std;

int main(int argc, char *argv[])
{
	if(argc < 2)
	{
		cout << "falta nombre de archivo de salida." << endl;
		return 0;
	}

	if(argc < 3)
	{
		cout << "falta fopen mode (w, a, r)." << endl;
		return 0;
	}

	string filename = argv[1];
	if(filename.find(".csv") == std::string::npos)
		filename = filename + ".csv";

	string fopen_mode = argv[2];
	FILE *fptr;
	fptr = fopen(("output/" + filename).c_str(),fopen_mode.c_str());

	if(fptr == NULL)
	{
		printf("Error!");   
		exit(1);             
	}

	if(fopen_mode.compare("w") == 0)
		fprintf(fptr,"dataset,data_type,size,resolution_scale,average_fps,low10_fps,low1_fps,std_dev,Mrps,average_jumps,internal_nodes,memory_usage,dimensionality\n");

	fclose(fptr);

//	system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 0 -m 2 -s 64 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 0 -m 2 -s 128 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 0 -m 2 -s 256 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 0 -m 2 -s 512 -bm -of " + filename).c_str());
//	//system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 0 -m 2 -s 1024 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 1 -m 2 -s 64 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 1 -m 2 -s 128 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 1 -m 2 -s 256 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 1 -m 2 -s 512 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 1 -m 2 -s 1024 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 3 -m 2 -s 64 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 3 -m 2 -s 128 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 3 -m 2 -s 256 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 3 -m 2 -s 512 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf teapot.stl -b teapot.txt -rx -90 -d 3 -m 2 -s 1024 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 0 -m 2 -s 64 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 0 -m 2 -s 128 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 0 -m 2 -s 256 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 0 -m 2 -s 512 -bm -of " + filename).c_str());
//	//system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 0 -m 2 -s 1024 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 1 -m 2 -s 64 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 1 -m 2 -s 128 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 1 -m 2 -s 256 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 1 -m 2 -s 512 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 1 -m 2 -s 1024 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 3 -m 2 -s 64 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 3 -m 2 -s 128 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 3 -m 2 -s 256 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 3 -m 2 -s 512 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf dragon.stl -b dragon.txt -d 3 -m 2 -s 1024 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 0 -m 2 -s 64 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 0 -m 2 -s 128 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 0 -m 2 -s 256 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 0 -m 2 -s 512 -bm -of " + filename).c_str());
//	//system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 0 -m 2 -s 1024 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 1 -m 2 -s 64 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 1 -m 2 -s 128 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 1 -m 2 -s 256 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 1 -m 2 -s 512 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 1 -m 2 -s 1024 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 3 -m 2 -s 64 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 3 -m 2 -s 128 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 3 -m 2 -s 256 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 3 -m 2 -s 512 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf bunny.stl  -b bunny.txt  -d 3 -m 2 -s 1024 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 0 -m 2 -s 64 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 0 -m 2 -s 128 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 0 -m 2 -s 256 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 0 -m 2 -s 512 -bm -of " + filename).c_str());
//	//system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 0 -m 2 -s 1024 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 1 -m 2 -s 64 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 1 -m 2 -s 128 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 1 -m 2 -s 256 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 1 -m 2 -s 512 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 1 -m 2 -s 1024 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 3 -m 2 -s 64 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 3 -m 2 -s 128 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 3 -m 2 -s 256 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 3 -m 2 -s 512 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf crab.stl   -b crab.txt   -rx -90 -d 3 -m 2 -s 1024 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 0 -m 2 -s 64 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 0 -m 2 -s 128 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 0 -m 2 -s 256 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 0 -m 2 -s 512 -bm -of " + filename).c_str());
//	//system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 0 -m 2 -s 1024 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 1 -m 2 -s 64 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 1 -m 2 -s 128 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 1 -m 2 -s 256 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 1 -m 2 -s 512 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 1 -m 2 -s 1024 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 3 -m 2 -s 64 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 3 -m 2 -s 128 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 3 -m 2 -s 256 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 3 -m 2 -s 512 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf city.stl   -b city.txt   -rx -90 -d 3 -m 2 -s 1024 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 0 -m 2 -s 64 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 0 -m 2 -s 128 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 0 -m 2 -s 256 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 0 -m 2 -s 512 -bm -of " + filename).c_str());
//	//system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 0 -m 2 -s 1024 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 1 -m 2 -s 64 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 1 -m 2 -s 128 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 1 -m 2 -s 256 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 1 -m 2 -s 512 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 1 -m 2 -s 1024 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 3 -m 2 -s 64 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 3 -m 2 -s 128 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 3 -m 2 -s 256 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 3 -m 2 -s 512 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf horse.stl  -b horse.txt  -rx -90 -d 3 -m 2 -s 1024 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 0 -m 2 -s 64 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 0 -m 2 -s 128 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 0 -m 2 -s 256 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 0 -m 2 -s 512 -bm -of " + filename).c_str());
//	//system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 0 -m 2 -s 1024 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 1 -m 2 -s 64 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 1 -m 2 -s 128 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 1 -m 2 -s 256 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 1 -m 2 -s 512 -bm -of " + filename).c_str());
	system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 1 -m 2 -s 1024 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 3 -m 2 -s 64 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 3 -m 2 -s 128 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 3 -m 2 -s 256 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 3 -m 2 -s 512 -bm -of " + filename).c_str());
//	system(("./main -ts 10 -mf cube.stl   -b cube.txt   -rx -90 -d 3 -m 2 -s 1024 -bm -of " + filename).c_str());

	return 0;
}