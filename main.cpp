#include <SDL2/SDL.h>

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <string>
#include <bitset>
#include <limits>
#include <chrono>
#include <iomanip>
#include <stack>
#include <fstream>
#include <sstream>

#include "SimplexNoise.h"

#include "render.hpp"



int main(int argc, char *argv[])
{
  RENDER_SCALE = 0.2f;
  BOUNDING_BOX_SIZE = 64;
  data_type = KDTREE;
  mesh_type = MESH;
  random_points_count = 100;	// only used if mesh_type is RANDOM
  noise_value_higher = 0.1f;
  noise_value_lower = -0.1f;
  mesh_filename = "teapot.stl";
  benchmark_name = "orbital.txt";
  output_filename = "output.csv";
  benchmark_mode = false;

  for(int i = 1; i < argc; i++)
    {
      if(strcmp(argv[i], "-bm") == 0)
	benchmark_mode = true;
      else
	{
	  if(strcmp(argv[i], "-b") == 0)
	    benchmark_name = argv[i + 1];
	  else if(strcmp(argv[i], "-r") == 0)
	    RENDER_SCALE = atof(argv[i + 1]);
	  else if(strcmp(argv[i], "-s") == 0)
	    BOUNDING_BOX_SIZE = atoi(argv[i + 1]);
	  else if(strcmp(argv[i], "-d") == 0)
	    data_type = (data_type_enum) atoi(argv[i + 1]);
	  else if(strcmp(argv[i], "-m") == 0)
	    mesh_type = (mesh_type_enum) atoi(argv[i + 1]);
	  else if(strcmp(argv[i], "-mf") == 0)
	    mesh_filename = argv[i + 1];
	  else if(strcmp(argv[i], "-p") == 0)
	    random_points_count = atoi(argv[i + 1]);
	  else if(strcmp(argv[i], "-of") == 0)
	    output_filename = argv[i + 1];
	  else if(strcmp(argv[i], "-h") == 0)
	    noise_value_higher = atof(argv[i + 1]);
	  else if(strcmp(argv[i], "-l") == 0)
	    noise_value_lower = atof(argv[i + 1]);
	  else if(strcmp(argv[i], "-rx") == 0)
	    rx = atof(argv[i + 1]);
	  else if(strcmp(argv[i], "-ry") == 0)
	    ry = atof(argv[i + 1]);
	  else if(strcmp(argv[i], "-rz") == 0)
	    rz = atof(argv[i + 1]);
	  else if(strcmp(argv[i], "-ts") == 0)
	    target_samples = atoi(argv[i + 1]);
	  else if(strcmp(argv[i], "-render") == 0)
	    render_mode = (render_mode_enum) atoi(argv[i + 1]);
			
	  i++;
	}
    }

  width_pixels = WIDTH * RENDER_SCALE;
  height_pixels = HEIGHT * RENDER_SCALE;
  tree_height = log(BOUNDING_BOX_SIZE) / log(2);
  lerp_speed_multiplier = BOUNDING_BOX_SIZE;

  window = SDL_CreateWindow("Voxel", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_RESIZABLE);
  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

  if(SDL_Init(SDL_INIT_VIDEO|SDL_INIT_AUDIO) != 0) {
    SDL_Log("Unable to initialize SDL: %s", SDL_GetError());
    return 1;
  }

  SDL_ShowCursor(SDL_DISABLE);

  screen = SDL_GetWindowSurface(window);
  pixels = SDL_CreateRGBSurfaceWithFormat(0, width_pixels, height_pixels, 32, SDL_PIXELFORMAT_RGBA32);
  pitch = pixels->pitch;

  SDL_GetClipRect(screen, &dst_rect);
  dst_rect.w /= RENDER_SCALE;
  dst_rect.h /= RENDER_SCALE;

  raycast_offset_multiplier = 2.0f * tan(0.5f * fov * 3.141592f / 180.0f);
  directional_light = normalize(directional_light);

  read_target_positions();
  initialize();

  run();
	
  if(!benchmark_mode)
    {
      chrono::steady_clock::time_point current_time = chrono::steady_clock::now();
      float duration = chrono::duration_cast<chrono::duration<float>>(current_time - raycasting_start).count();

      float average_fps = 0;
      int total_samples = 0;
      for(int i = 0; i < 120; i++)
	{
	  average_fps += i * fps[i];
	  total_samples += fps[i];
	}
      average_fps /= total_samples;
      std::cout << "total samples: " << total_samples << std::endl;
      std::cout << "average fps: " << average_fps << std::endl;

      int p10_samples = std::max(1, total_samples / 10);	//percentile 1
      int samples_considered = 0;
      float average_p10_fps = 0;
      for(int i = 0; i < 120; i++)
	{
	  int samples = std::min(fps[i], p10_samples - samples_considered);
	  samples_considered += samples;
	  average_p10_fps += i * samples;
	}
      average_p10_fps /= (float) p10_samples;
      std::cout << "average 10% low fps: " << average_p10_fps << std::endl;

      int p1_samples = std::max(1, total_samples / 100);	//percentile 1
      samples_considered = 0;
      float average_p1_fps = 0;
      for(int i = 0; i < 120; i++)
	{
	  int samples = std::min(fps[i], p1_samples - samples_considered);
	  samples_considered += samples;
	  average_p1_fps += i * samples;
	}
      average_p1_fps /= (float) p1_samples;
      std::cout << "average 1% low fps: " << average_p1_fps << std::endl;

      float standart_deviation = 0;
      for(int i = 0; i < 120; i++)
	standart_deviation += fps[i] * pow(i - average_fps, 2);
      standart_deviation /= total_samples;
      standart_deviation = sqrt(standart_deviation);
      std::cout << "standart_deviation: " << standart_deviation << std::endl;

      std::cout << "Mrays per second: " << rays / duration / 1000000.0f << std::endl;

      std::cout << "average jumps: " << average_jumps / (float) total_frames / (float) width_pixels / (float) height_pixels << std::endl;

      cout << "memory usage: " << print_memory_usage() << " bytes" << endl;
      cout << "voxels: " << solid_nodes[tree_height] << endl;
    }
  else
    {
      chrono::steady_clock::time_point current_time = chrono::steady_clock::now();
      float duration = chrono::duration_cast<chrono::duration<float>>(current_time - raycasting_start).count();

      float average_fps = 0;
      int total_samples = 0;
      for(int i = 0; i < 120; i++)
	{
	  average_fps += i * fps[i];
	  total_samples += fps[i];
	}
      average_fps /= total_samples;

      int p10_samples = std::max(1, total_samples / 10);	//percentile 1
      int samples_considered = 0;
      float average_p10_fps = 0;
      for(int i = 0; i < 120; i++)
	{
	  int samples = std::min(fps[i], p10_samples - samples_considered);
	  samples_considered += samples;
	  average_p10_fps += i * samples;
	}
      average_p10_fps /= (float) p10_samples;

      int p1_samples = std::max(1, total_samples / 100);	//percentile 1
      samples_considered = 0;
      float average_p1_fps = 0;
      for(int i = 0; i < 120; i++)
	{
	  int samples = std::min(fps[i], p1_samples - samples_considered);
	  samples_considered += samples;
	  average_p1_fps += i * samples;
	}
      average_p1_fps /= (float) p1_samples;

      float standart_deviation = 0;
      for(int i = 0; i < 120; i++)
	standart_deviation += fps[i] * pow(i - average_fps, 2);
      standart_deviation /= total_samples;
      standart_deviation = sqrt(standart_deviation);

      if(output_filename.find(".csv") == std::string::npos)
	output_filename = output_filename + ".csv";

      FILE *fptr;
      fptr = fopen(("output/" + output_filename).c_str(),"a");

      if(fptr == NULL)
	{
	  printf("Error!");   
	  exit(1);             
	}

      string data_type_names[] = {"matrix", "octree", "octree_stack", "k3-tree", "k3-tree_lidar"};
	
      string extension = ".stl";
      string name = mesh_filename;
      std::size_t ind = name.find(extension); // Find the starting position of substring in the string
      if(ind !=std::string::npos)
	name.erase(ind,extension.length());

      fprintf(fptr,"%s,%s,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s\n",
	      name.c_str(),
	      data_type_names[data_type].c_str(),
	      BOUNDING_BOX_SIZE,
	      RENDER_SCALE,
	      average_fps,
	      average_p10_fps,
	      average_p1_fps,
	      standart_deviation,
	      rays / duration / 1000000.0f,
	      average_jumps / (float) total_frames / (float) width_pixels / (float) height_pixels,
	      print_memory_usage().c_str()
	      );

      fclose(fptr);
    }

  SDL_FreeSurface(pixels);
  SDL_FreeSurface(screen);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
