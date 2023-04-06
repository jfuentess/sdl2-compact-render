#include <SDL2/SDL.h>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <windows.h>
#include <conio.h>
#include <bitset>

#define WIDTH 1600
#define HEIGHT 900
#define RENDER_SCALE 0.1
#define FRAMERATE 120

using namespace std;

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

class vec3 {
public:
	double x, y, z;
	
	vec3() :x(0), y(0), z(0) {}
	vec3(double x, double y, double z) : x(x), y(y), z(z) {}
	vec3(const vec3& v) : x(v.x), y(v.y), z(v.z) {}
	
	vec3& operator=(const vec3 v) {
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}
	
	//vec3 operator+(vec3& v) {
	//	return vec3(x + v.x, y + v.y, z + v.z);
	//}
	//vec3 operator-(vec3& v) {
	//	return vec3(x - v.x, y - v.y, z - v.z);
	//}
	
	vec3 operator+(vec3 v) {
		return vec3(x + v.x, y + v.y, z + v.z);
	}
	vec3 operator-(vec3 v) {
		return vec3(x - v.x, y - v.y, z - v.z);
	}
	
	vec3& operator+=(vec3 v) {
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	vec3& operator-=(vec3 v) {
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}

	vec3 operator*(double s) {
		return vec3(x * s, y * s, z * s);
	}
	vec3 operator/(double s) {
		return vec3(x / s, y / s, z / s);
	}
	
	vec3& operator+=(double s) {
		x += s;
		y += s;
		z += s;
		return *this;
	}
	vec3& operator-=(double s) {
		x -= s;
		y -= s;
		z -= s;
		return *this;
	}
	vec3& operator*=(double s) {
		x *= s;
		y *= s;
		z *= s;
		return *this;
	}
	vec3& operator/=(double s) {
		x /= s;
		y /= s;
		z /= s;
		return *this;
	}
	
	void set(double x, double y, double z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}
	
	void round() {
		this->x = std::round(x);
		this->y = std::round(y);
		this->z = std::round(z);
	}
	
	//void rotate(double deg) {
	//	double theta = deg / 180.0 * M_PI;
	//	double c = cos(theta);
	//	double s = sin(theta);
	//	double tx = x * c - y * s;
	//	double ty = x * s + y * c;
	//	x = tx;
	//	y = ty;
	//}
	
	vec3& normalize() {
		if (length() == 0) return *this;
		*this *= (1.0 / length());
		return *this;
	}
	
	double dist(vec3 v) const {
		vec3 d(v.x - x, v.y - y, v.z - z);
		return d.length();
	}
	double length() const {
		return std::sqrt(x * x + y * y + z * z);
	}
	//void truncate(double length) {
	//	double angle = atan2f(y, x);
	//	x = length * cos(angle);
	//	y = length * sin(angle);
	//}
	
	//vec3 ortho() const {
	//	return vec3(y, -x);
	//}
	
	static double dot(vec3 v1, vec3 v2) {
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	}

	static vec3 cross(vec3 v1, vec3 v2) {
		return vec3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
	}
	
};

//	SDL variables
SDL_Window *window = SDL_CreateWindow("VOXEL", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_RESIZABLE);
SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

//	data variables
int size = 256;
int*** data;
vec3 bounding_box_start;
vec3 bounding_box_end;

// math
vec3 world_up = vec3(0, 1, 0);

// camera
vec3 camera_pos;
vec3 camera_right;
vec3 camera_up;
vec3 camera_forward;
double camera_x_angle;
double camera_y_angle;
double mouse_sens = 0.2;
bool draw_again = false;
bool hide_mouse = true;
char pixel_scale;
int width_pixels = WIDTH * RENDER_SCALE;
int height_pixels = HEIGHT * RENDER_SCALE;

vec3 front_voxel;
vec3 adjacent_voxel;

//	input variables
vec3 mouse_pos;
bool input_quit = false;
bool input_lclick = false;
bool input_lclick_down = false;
bool input_lclick_down_reset = true;
bool input_rclick = false;
bool input_rclick_down = false;
bool input_rclick_down_reset = true;

bool input_forward = false;
bool input_left = false;
bool input_back = false;
bool input_right = false;
bool input_up = false;
bool input_down = false;
bool input_shift = false;
bool input_position = false;
bool input_recreate = false;

// colors
SDL_Color colors[] = {
	{115, 70, 76},
	{171, 86, 117},
	{238, 106, 124},
	{255, 167, 165},
	{255, 224, 126},
	{255, 231, 214},
	{114, 220, 187},
	{52, 172, 186},
	{115, 70, 76},
	{171, 86, 117},
	{238, 106, 124},
	{255, 167, 165},
	{255, 224, 126},
	{255, 231, 214},
	{114, 220, 187},
	{52, 172, 186},
	{115, 70, 76},
	{171, 86, 117},
	{238, 106, 124},
	{255, 167, 165},
	{255, 224, 126},
	{255, 231, 214},
	{114, 220, 187},
	{52, 172, 186},
	{115, 70, 76},
	{171, 86, 117},
	{238, 106, 124},
	{255, 167, 165},
	{255, 224, 126},
	{255, 231, 214},
	{114, 220, 187},
	{52, 172, 186}
};

int arr[4];

void redraw()
{
	draw_again = true;
}

char get_voxel(int i, int j, int k)
{
	return data[i][j][k];
}

void compute_raycast()
{
	for (int i = 0; i < width_pixels; ++i)
	{
		for (int j = 0; j < height_pixels; ++j)
		{
			vec3 right_offset = vec3(camera_right);
			right_offset *= ((i - width_pixels / 2.0) / width_pixels) * 2.0;

			vec3 up_offset = vec3(camera_up);
			up_offset *= ((j - height_pixels / 2.0) / width_pixels) * 2.0;

			vec3 current_dir = vec3(camera_forward + right_offset + up_offset);
			current_dir += right_offset;
			current_dir += up_offset;

			vec3 step = vec3();
			vec3 deltaT = vec3();
			vec3 distance = vec3();

			if(current_dir.x != 0)
			{
				step.x = current_dir.x < 0 ? -1 : 1;
				deltaT.x = abs(1.0 / current_dir.x);
				distance.x = step.x < 0 ? (floor(camera_pos.x) - camera_pos.x) / current_dir.x : (ceil(camera_pos.x) - camera_pos.x) / current_dir.x;
			}
			else
			{
				deltaT.x = 99999;
				distance.x = 99999;
			}

			if(current_dir.y != 0){
				step.y = current_dir.y < 0 ? -1 : 1;
				deltaT.y = abs(1.0 / current_dir.y);
				distance.y = step.y < 0 ? (floor(camera_pos.y) - camera_pos.y) / current_dir.y : (ceil(camera_pos.y) - camera_pos.y) / current_dir.y;
			}
			else
			{
				deltaT.y = 99999;
				distance.y = 99999;
			}

			if(current_dir.z != 0){
				step.z = current_dir.z < 0 ? -1 : 1;
				deltaT.z = abs(1.0 / current_dir.z);
				distance.z = step.z < 0 ? (floor(camera_pos.z) - camera_pos.z) / current_dir.z : (ceil(camera_pos.z) - camera_pos.z) / current_dir.z;
			}
			else
			{
				deltaT.z = 99999;
				distance.z = 99999;
			}

			vec3 current_voxel = vec3(camera_pos);
			current_voxel += 0.5;
			current_voxel.round();

			vec3 voxel_incr = vec3();

			while(current_voxel.x > bounding_box_start.x && current_voxel.y > bounding_box_start.y && current_voxel.z > bounding_box_start.z &&
				current_voxel.x < bounding_box_end.x && current_voxel.y < bounding_box_end.y && current_voxel.z < bounding_box_end.z)
			{
				double total_dist = 9999;
				if(distance.x < total_dist)
					total_dist = distance.x;
				if(distance.y < total_dist)
					total_dist = distance.y;
				if(distance.z < total_dist)
					total_dist = distance.z;

				//adjacent_voxel = vec3(-step.x, -step.y, -step.z);

				voxel_incr.x = (distance.x <= distance.y) && (distance.x <= distance.z);
				voxel_incr.y = (distance.y <= distance.x) && (distance.y <= distance.z);
				voxel_incr.z = (distance.z <= distance.x) && (distance.z <= distance.y);

				distance.x += voxel_incr.x * deltaT.x;
				distance.y += voxel_incr.y * deltaT.y;
				distance.z += voxel_incr.z * deltaT.z;

				current_voxel.x += voxel_incr.x * step.x;
				current_voxel.y += voxel_incr.y * step.y;
				current_voxel.z += voxel_incr.z * step.z;

				if(current_voxel.x <= bounding_box_start.x || current_voxel.y <= bounding_box_start.y || current_voxel.z <= bounding_box_start.z ||
				current_voxel.x >= bounding_box_end.x || current_voxel.y >= bounding_box_end.y || current_voxel.z >= bounding_box_end.z)
					break;

				int voxel = get_voxel((int) (current_voxel.x + size / 2.0 - 0.5), (int) (current_voxel.y + size / 2.0 - 0.5), (int) (current_voxel.z + size / 2.0 - 0.5));
				if(voxel > 0)
				{
					if(i == width_pixels / 2 && j == height_pixels / 2)
						front_voxel.set((int) (current_voxel.x + size / 2.0 - 0.5), (int) (current_voxel.y + size / 2.0 - 0.5), (int) (current_voxel.z + size / 2.0 - 0.5));

					//SDL_SetRenderDrawColor(renderer,
					//	max(0, (int) (colors[voxel - 1].r - total_dist * 10)),
					//	max(0, (int) (colors[voxel - 1].g - total_dist * 10)),
					//	max(0, (int) (colors[voxel - 1].b - total_dist * 10)),
					//	SDL_ALPHA_OPAQUE);

					SDL_SetRenderDrawColor(renderer,
						min(255, max(0, (int) (total_dist * 20))),
						min(255, max(0, (int) (total_dist * 20))),
						min(255, max(0, (int) (total_dist * 20))),
						SDL_ALPHA_OPAQUE);

					SDL_RenderDrawPoint(renderer, i, (height_pixels - 1 - j));
					break;
				}
			}
		}
	}
}

void draw()
{
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
	//SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

	SDL_RenderClear(renderer);

	compute_raycast();

	SDL_RenderPresent(renderer);

}

void initialize()
{
	pixel_scale = ceil(1 / RENDER_SCALE);

	bounding_box_start = vec3(-size / 2.0, -size / 2.0, -size / 2.0);
	bounding_box_end = vec3(size / 2.0, size / 2.0, size / 2.0);

	data = new int**[size];
	for(int i = 0; i < size; i++)
	{
		data[i] = new int*[size];
		for(int j = 0; j < size; j++)
		{
			data[i][j] = new int[size];
			for(int k = 0; k < size; k++)
			{
				data[i][j][k] = (rand() % 5 == 0) ? (rand() % 8) + 1 : 0;
			}
		}
	}

	if(!input_recreate)
	{
		camera_pos = vec3(0.5, 0.5, - size / 2.0 + 0.5);
		camera_forward.set(0, 0, 1);
		camera_right = vec3::cross(camera_forward, world_up);
		camera_up = vec3::cross(camera_right, camera_forward);
		camera_x_angle = 0;
		camera_y_angle = 0;

		redraw();
	}
}

void update()
{
	if(input_position)
	{
		camera_pos = vec3(0.5, 0.5, - size / 2.0 + 0.5);
		camera_forward.set(0, 0, 1);
		camera_right = vec3::cross(camera_forward, world_up);
		camera_up = vec3::cross(camera_right, camera_forward);
		camera_x_angle = 0;
		camera_y_angle = 0;
		input_position = false;

		redraw();
	}
	
	if(input_recreate)
	{
		initialize();
		input_recreate = false;
	}

	if(input_lclick_down)
		data[(int) front_voxel.x][(int) front_voxel.y][(int) front_voxel.z] = 0;

	if(input_rclick_down)
		data[(int) (front_voxel.x + adjacent_voxel.x)][(int) (front_voxel.y + adjacent_voxel.y)][(int) (front_voxel.z + adjacent_voxel.z)] = (rand() % 8) + 1;

	if(input_forward)
	{
		vec3 add = vec3(camera_forward);
		add *= 0.01666;
		camera_pos += add;
		redraw();
	}
	if(input_left)
	{
		vec3 add = vec3(camera_right);
		add *= 0.01666;
		camera_pos -= add;
		redraw();
	}
	if(input_back )
	{
		vec3 add = vec3(camera_forward);
		add *= 0.01666;
		camera_pos -= add;
		redraw();
	}
	if(input_right)
	{
		vec3 add = vec3(camera_right);
		add *= 0.01666;
		camera_pos += add;
		redraw();
	}
	if(input_up)
	{
		vec3 add = vec3(camera_up);
		add *= 0.01666;
		camera_pos += add;
		redraw();
	}
	if(input_down)
	{
		vec3 add = vec3(camera_up);
		add *= 0.01666;
		camera_pos -= add;
		redraw();
	}

	if(hide_mouse)
		SDL_WarpMouseInWindow(window, WIDTH / 2, HEIGHT / 2);
}

void catch_input()
{
	input_lclick_down = false;
	input_rclick_down = false;
	SDL_Event event;
	while(SDL_PollEvent(&event)){
		if(event.type == SDL_QUIT){
			input_quit = true;
		}
		else if(event.type == SDL_KEYDOWN){
			if(event.key.keysym.scancode == SDL_SCANCODE_ESCAPE)
			{
				hide_mouse = !hide_mouse;
				if(hide_mouse)
					SDL_ShowCursor(SDL_DISABLE);
				else
					SDL_ShowCursor(SDL_ENABLE);
			}
			else if(event.key.keysym.scancode == SDL_SCANCODE_W) input_forward = true;
			else if(event.key.keysym.scancode == SDL_SCANCODE_A) input_left = true;
			else if(event.key.keysym.scancode == SDL_SCANCODE_S) input_back = true;
			else if(event.key.keysym.scancode == SDL_SCANCODE_D) input_right = true;
			else if(event.key.keysym.scancode == SDL_SCANCODE_E) input_up = true;
			else if(event.key.keysym.scancode == SDL_SCANCODE_Q) input_down = true;
			else if(event.key.keysym.scancode == SDL_SCANCODE_LSHIFT) input_shift = true;
			else if(event.key.keysym.scancode == SDL_SCANCODE_R) input_recreate = true;
			else if(event.key.keysym.scancode == SDL_SCANCODE_H) input_position = true;
		}
		else if(event.type == SDL_KEYUP){
			if(event.key.keysym.scancode == SDL_SCANCODE_ESCAPE)
			{

			}
			else if(event.key.keysym.scancode == SDL_SCANCODE_W) input_forward = false;
			else if(event.key.keysym.scancode == SDL_SCANCODE_A) input_left = false;
			else if(event.key.keysym.scancode == SDL_SCANCODE_S) input_back = false;
			else if(event.key.keysym.scancode == SDL_SCANCODE_D) input_right = false;
			else if(event.key.keysym.scancode == SDL_SCANCODE_E) input_up = false;
			else if(event.key.keysym.scancode == SDL_SCANCODE_Q) input_down = false;
			else if(event.key.keysym.scancode == SDL_SCANCODE_LSHIFT) input_shift = false;
			else if(event.key.keysym.scancode == SDL_SCANCODE_R) input_recreate = false;
			else if(event.key.keysym.scancode == SDL_SCANCODE_H) input_position = false;
		}
		else if(event.type == SDL_MOUSEMOTION)
		{
			camera_x_angle = min(89.9, max(-89.9, camera_x_angle - event.motion.yrel * mouse_sens));
			camera_y_angle -= event.motion.xrel * mouse_sens;
			if(camera_y_angle < -180)
				camera_y_angle += 360;
			if(camera_y_angle >= 180)
				camera_y_angle -= 360;

			camera_forward.set(0, 0, 1);

			double x, y, z, rad;

			rad = camera_x_angle * M_PI / 180.0;
			z = camera_forward.z;
			y = camera_forward.y;
			camera_forward.z = z * cos(rad) - y * sin(rad);
			camera_forward.y = z * sin(rad) + y * cos(rad);

			rad = camera_y_angle * M_PI / 180.0;
			z = camera_forward.z;
			x = camera_forward.x;
			camera_forward.z = z * cos(rad) - x * sin(rad);
			camera_forward.x = z * sin(rad) + x * cos(rad);

			camera_right = vec3::cross(camera_forward, world_up).normalize();
			camera_up = vec3::cross(camera_right, camera_forward);

			redraw();
		}
		else if(event.type == SDL_MOUSEBUTTONDOWN)
		{
			if(event.button.button == SDL_BUTTON_LEFT)
			{
				input_lclick_down = input_lclick_down_reset;
				input_lclick = true;
				input_lclick_down_reset = false;
			}
			if(event.button.button == SDL_BUTTON_RIGHT)
			{
				input_rclick_down = input_rclick_down_reset;
				input_rclick = true;
				input_rclick_down_reset = false;
			}
		}
		else if(event.type == SDL_MOUSEBUTTONUP)
		{
			if(event.button.button == SDL_BUTTON_LEFT)
			{
				input_lclick_down_reset = true;
				input_lclick = false;
			}
			if(event.button.button == SDL_BUTTON_RIGHT)
			{
				input_rclick_down_reset = true;
				input_rclick = false;
			}
		}
	}
}

void run()
{
	while(!input_quit)
	{
		catch_input();
		update();
		if(draw_again)
		{
			draw();
			draw_again = false;
		}
	}
}

int main(int argc, char *argv[])
{
	srand(time(NULL));

	if(SDL_Init(SDL_INIT_VIDEO|SDL_INIT_AUDIO) != 0) {
		SDL_Log("Unable to initialize SDL: %s", SDL_GetError());
		return 1;
	}
	SDL_ShowCursor(SDL_DISABLE);
	SDL_RenderSetScale(renderer, 1.0 / RENDER_SCALE, 1.0 / RENDER_SCALE);

	initialize();
	run();

	SDL_Quit();
	return 0;
}