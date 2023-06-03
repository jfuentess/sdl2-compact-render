#define NOMINMAX

#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/projection.hpp>

#include <SDL2/SDL.h>

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <string>
#include <windows.h>
#include <conio.h>
#include <bitset>
#include <limits>
#include <chrono>
#include <iomanip>
#include <stack>

#include "SimplexNoise.h"
#include "stl_reader.h"

#define WIDTH 1280
#define HEIGHT 720
#define FRAMERATE 120
#define SMOOTHSTEP_MIN 5
#define SMOOTHSTEP_MAX 15
using namespace std;
using namespace glm;
using namespace stl_reader;

enum data_type_enum
{
	MATRIX,
	OCTREE,
	KDTREE
};

enum mesh_type_enum
{
	NOISE,
	MESH,
	RANDOM
};

enum render_mode_enum
{
	COLOR,
	DEPTH,
	JUMPS,
	NOT_NULL_JUMPS
};

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

// Octree node struct
struct node_t
{
	bool leaf;
	node_t* children[8];
	vec4 voxel_color;
};

// Octree struct
struct octree_t
{
	int size;
	int depth;
	node_t* root;
};

// Stack node item struct
struct stack_node_t
{
	node_t* node;
	vec3 position;
};

//	SDL variables
SDL_Window *window = SDL_CreateWindow("Voxel", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_RESIZABLE);
SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
SDL_Surface * screen = nullptr;
SDL_Surface * pixels = nullptr;

std::chrono::steady_clock::time_point previous_time = std::chrono::steady_clock::now();
float delta_time = 0.0f;
float smooth_delta_time = 0.0f;

//	render variables
int pitch;
SDL_Rect dst_rect;

//	voxel_data variables
vec3 bounding_box;

vec4* voxel_data;
octree_t* octree;

data_type_enum data_type;
mesh_type_enum mesh_type;
int random_points_count;	// in case mesh_type is RANDOM

float RENDER_SCALE;
int BOUNDING_BOX_SIZE;

// math
vec3 world_up = vec3(0, 1, 0);

// raycast
float start_node_size;
vec3 start_proportional_position;
std::stack<stack_node_t> node_stack;
std::stack<stack_node_t> default_stack;
int jumps;
int not_null_jumps = 0;
int max_depth = 0;
int jump_multiplier = 0;
int blue_and_green;
render_mode_enum render_mode = COLOR;

// camera
vec3 camera_pos;
vec3 camera_right;
vec3 camera_up;
vec3 camera_forward;
vec2 camera_angles;
float fov = 70;
float raycast_offset_multiplier;
vec3 directional_light = normalize(vec3(-1.0f, -1.5f, -1.3f));
bool outside;

float mouse_sens = 0.2;
bool draw_again = false;
bool hide_mouse = true;
char pixel_scale;
int width_pixels;
int height_pixels;

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
vec4 current_color;
SDL_Color colors[] = {
	{115, 70, 76},
	{171, 86, 117},
	{238, 106, 124},
	{255, 167, 165},
	{255, 224, 126},
	{255, 231, 214},
	{114, 220, 187},
	{52, 172, 186}
};

void redraw()
{
	draw_again = true;
}

unsigned int hex(vec4 color)
{
	return ((int(color.w) & 0xff) << 24) + ((int(color.z) & 0xff) << 16) + ((int(color.y) & 0xff) << 8) + (int(color.x) & 0xff);
}

void set_pixel(int i, int j, vec4 color)
{
	unsigned int* row = (unsigned int*) ((char*) pixels->pixels + pitch * (height_pixels - 1 - j));
	row[i] = hex(color);
	//SDL_SetRenderDrawColor(renderer, color.x, color.y, color.z, color.w);
	//SDL_RenderDrawPoint(renderer, i, (height_pixels - 1 - j));
}

float clamp(float x, float lowerlimit, float upperlimit) {
	if (x < lowerlimit)
		x = lowerlimit;
	if (x > upperlimit)
		x = upperlimit;
	return x;
}

float smootherstep(float edge0, float edge1, float x) {
	// Scale, and clamp x to 0..1 range
	x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
	// Evaluate polynomial
	return x * x * x * (x * (x * 6 - 15) + 10);
}

vec4 get_background_pixel(vec3 ray_direction)
{
	float brightness = (1.0f / pow(abs(abs(ray_direction.x) - abs(ray_direction.y)), 0.5f) + 1.0f / pow(abs(abs(ray_direction.x) - abs(ray_direction.z)), 0.5f)) * (float) (abs(ray_direction.x) > abs(ray_direction.y) && abs(ray_direction.x) > abs(ray_direction.z));
	brightness += (1.0f / pow(abs(abs(ray_direction.x) - abs(ray_direction.y)), 0.5f) + 1.0f / pow(abs(abs(ray_direction.y) - abs(ray_direction.z)), 0.5f)) * (float) (abs(ray_direction.y) > abs(ray_direction.x) && abs(ray_direction.y) > abs(ray_direction.z));
	brightness += (1.0f / pow(abs(abs(ray_direction.z) - abs(ray_direction.x)), 0.5f) + 1.0f / pow(abs(abs(ray_direction.z) - abs(ray_direction.y)), 0.5f)) * (float) (abs(ray_direction.z) > abs(ray_direction.y) && abs(ray_direction.z) > abs(ray_direction.x));
	brightness = smootherstep(SMOOTHSTEP_MIN, SMOOTHSTEP_MAX, brightness) * 64;
	return vec4(brightness, brightness, brightness, 255);
}

vec4 rgb2hsv(vec4 in)
{
	in = vec4(in.x / 255.0, in.y / 255.0, in.z / 255.0, in.w / 255.0);

	vec4 out;
	out.w = in.w;

	double min, max, delta;

	min = in.x < in.y ? in.x : in.y;
	min = min  < in.z ? min  : in.z;

	max = in.x > in.y ? in.x : in.y;
	max = max  > in.z ? max  : in.z;

	out.z = max;                                // v
	delta = max - min;
	if (delta < 0.00001)
	{
		out.y = 0;
		out.x = 0; // undefined, maybe nan?
		return out * 255.0f;
	}
	if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
		out.y = (delta / max);                  // s
	} else {
		// if max is 0, then r = g = b = 0              
		// s = 0, h is undefined
		out.y = 0.0;
		out.x = 0.0;                       		// its now undefined
		return out * 255.0f;
	}
	if( in.x >= max )                           // > is bogus, just keeps compilor happy
		out.x = ( in.y - in.z ) / delta;        // between yellow & magenta
	else
	if( in.y >= max )
		out.x = 2.0 + ( in.z - in.x ) / delta;  // between cyan & yellow
	else
		out.x = 4.0 + ( in.x - in.y ) / delta;  // between magenta & cyan

	out.x *= 60.0;                              // degrees

	if( out.x < 0.0 )
		out.x += 360.0;

	return out * 255.0f;
}

vec4 hsv2rgb(vec4 in)
{
	in = vec4(in.x / 255.0, in.y / 255.0, in.z / 255.0, in.w / 255.0);

	double hh, p, q, t, ff;
	long i;
	vec4 out;
	out.w = in.w;

	if(in.y <= 0.0) {       // < is bogus, just shuts up warnings
		out.x = in.z;
		out.y = in.z;
		out.z = in.z;
		return out * 255.0f;
	}
	hh = in.x;
	if(hh >= 1.0) hh = 0.0;
	hh *= 6.0;
	i = (long)hh;
	ff = hh - i;
	p = in.z * (1.0 - in.y);
	q = in.z * (1.0 - (in.y * ff));
	t = in.z * (1.0 - (in.y * (1.0 - ff)));

	switch(i) {
	case 0:
		out.x = in.z;
		out.y = t;
		out.z = p;
		break;
	case 1:
		out.x = q;
		out.y = in.z;
		out.z = p;
		break;
	case 2:
		out.x = p;
		out.y = in.z;
		out.z = t;
		break;

	case 3:
		out.x = p;
		out.y = q;
		out.z = in.z;
		break;
	case 4:
		out.x = t;
		out.y = p;
		out.z = in.z;
		break;
	case 5:
	default:
		out.x = in.z;
		out.y = p;
		out.z = q;
		break;
	}
	return out * 255.0f;
}

vec4 get_matrix_voxel(vec3 v)
{
	if((int) v.x >= 0 && (int) v.x < BOUNDING_BOX_SIZE && (int) v.y >= 0 && (int) v.y < BOUNDING_BOX_SIZE && (int) v.z >= 0 && (int) v.z < BOUNDING_BOX_SIZE)
		return voxel_data[(int) v.x * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE + (int) v.y * BOUNDING_BOX_SIZE + (int) v.z];
	return vec4(0, 0, 0, 0);
}

bool ray_intersects_aabb(vec3 o, vec3 d, vec3 bb, vec3* intersection_point, vec3* face_normal)
{
	float t_min = numeric_limits<float>::min();
	float t_max = numeric_limits<float>::max();
	vec3 p = bb * 0.5f - o;
	float h = bb.x * 0.5f;
	vec3 normal;

	for (int i = 0; i < 3; i++)
	{
		float e = i == 0 ? p.x : i == 1 ? p.y : p.z;
		float f = i == 0 ? d.x : i == 1 ? d.y : d.z;
		if(f != 0)
		{
			float t1 = (e + h) / f;
			float t2 = (e - h) / f;
			if(t1 > t2)
			{
				float aux = t1;
				t1 = t2;
				t2 = aux;
			}
			if(t1 > t_min)
			{
				normal = - vec3(float(i == 0) * (d.x < 0 ? -1 : 1), float(i == 1) * (d.y < 0 ? -1 : 1), float(i == 2) * (d.z < 0 ? -1 : 1));
				t_min = t1;
			}
			if(t2 < t_max)
				t_max = t2;
			if(t_min > t_max) return false;
			if(t_max < 0) return false;
		}
		else if(- e - h > 0 || - e + h < 0) return false;
	}
	if(t_min > 0)
	{
		*intersection_point = o + d * t_min;
		*face_normal = normal;
		return true;
	}
	return false;
}

void raycast_matrix()
{
	for (int i = 0; i < width_pixels; ++i)
	{
		for (int j = 0; j < height_pixels; ++j)
		{
			jumps = 0;
			not_null_jumps = 0;
			vec3 right_offset = vec3(camera_right);
			right_offset *= ((i - width_pixels / 2.0) / height_pixels) * raycast_offset_multiplier;

			vec3 up_offset = vec3(camera_up);
			up_offset *= ((j - height_pixels / 2.0) / height_pixels) * raycast_offset_multiplier;

			vec3 ray_direction = normalize(camera_forward + right_offset + up_offset);

			float total_dist = 0;

			vec3 ray_origin = camera_pos;
			vec3 face_normal = vec3();

			if(outside)
			{
				if(ray_intersects_aabb(camera_pos, ray_direction, bounding_box, &ray_origin, &face_normal))
				{
					total_dist = distance(camera_pos, ray_origin);
					vec4 voxel = get_matrix_voxel(ray_origin - face_normal * 0.00001f);
					ray_origin -= face_normal * 0.00001f;
					if(voxel.w > 0)
					{
						if(render_mode == DEPTH)
						{
							current_color = vec4(
								std::min(255, std::max(0, (int) (total_dist * 250.0f / BOUNDING_BOX_SIZE))),
								std::min(255, std::max(0, (int) (total_dist * 250.0f / BOUNDING_BOX_SIZE))),
								std::min(255, std::max(0, (int) (total_dist * 250.0f / BOUNDING_BOX_SIZE))),
								255);
						}
						else if(render_mode == COLOR)
						{
							current_color = vec4(
								std::max(0, std::min(255, (int) (voxel.x * (2.0f / (1 + exp(total_dist / 64.0f)))))),
								std::max(0, std::min(255, (int) (voxel.y * (2.0f / (1 + exp(total_dist / 64.0f)))))),
								std::max(0, std::min(255, (int) (voxel.z * (2.0f / (1 + exp(total_dist / 64.0f)))))),
								255);
							//current_color *= float(dot(-face_normal, directional_light) * 0.25 + 0.75);
							current_color.w = 255;
						}
						else if(render_mode == JUMPS)
						{
							blue_and_green = std::max(0, 255 - (max_depth + jumps) * jump_multiplier * 2);
							current_color = vec4(255, blue_and_green, blue_and_green, 255);
						}
						else if(render_mode == NOT_NULL_JUMPS)
						{
							blue_and_green = std::max(0, 255 - max_depth * jump_multiplier * 4);
							current_color = vec4(blue_and_green, blue_and_green, 255, 255);
						}
						set_pixel(i, j, current_color);
						continue;
					}
					jumps++;
				}
				else
				{
					blue_and_green = std::max(0, 255 - jumps * jump_multiplier);
					set_pixel(i, j, vec4(255, blue_and_green, blue_and_green, 255));
					//set_pixel(i, j, get_background_pixel(ray_direction));
					continue;
				}
			}

			// traversal
			vec3 step = vec3();
			vec3 deltaT = vec3();
			vec3 distance = vec3(total_dist);

			if(ray_direction.x != 0)
			{
				step.x = ray_direction.x < 0 ? -1 : 1;
				deltaT.x = abs(1.0 / ray_direction.x);
				distance.x += step.x < 0 ? (floor(ray_origin.x) - ray_origin.x) / ray_direction.x : (ceil(ray_origin.x) - ray_origin.x) / ray_direction.x;
			}
			else
			{
				deltaT.x = 99999;
				distance.x = 99999;
			}

			if(ray_direction.y != 0){
				step.y = ray_direction.y < 0 ? -1 : 1;
				deltaT.y = abs(1.0 / ray_direction.y);
				distance.y += step.y < 0 ? (floor(ray_origin.y) - ray_origin.y) / ray_direction.y : (ceil(ray_origin.y) - ray_origin.y) / ray_direction.y;
			}
			else
			{
				deltaT.y = 99999;
				distance.y = 99999;
			}

			if(ray_direction.z != 0){
				step.z = ray_direction.z < 0 ? -1 : 1;
				deltaT.z = abs(1.0 / ray_direction.z);
				distance.z += step.z < 0 ? (floor(ray_origin.z) - ray_origin.z) / ray_direction.z : (ceil(ray_origin.z) - ray_origin.z) / ray_direction.z;
			}
			else
			{
				deltaT.z = 99999;
				distance.z = 99999;
			}

			vec3 current_voxel = floor(vec3(ray_origin));
			vec3 voxel_incr;

			while(total_dist < 9999)
			{
				total_dist = 9999;
				if(distance.x < total_dist)
					total_dist = distance.x;
				if(distance.y < total_dist)
					total_dist = distance.y;
				if(distance.z < total_dist)
					total_dist = distance.z;

				voxel_incr.x = (distance.x <= distance.y) && (distance.x <= distance.z);
				voxel_incr.y = (distance.y <= distance.x) && (distance.y <= distance.z);
				voxel_incr.z = (distance.z <= distance.x) && (distance.z <= distance.y);

				distance += voxel_incr * deltaT;
				current_voxel += voxel_incr * step;
				jumps++;

				if(current_voxel.x < 0 || current_voxel.y < 0 || current_voxel.z < 0 ||
				current_voxel.x >= bounding_box.x || current_voxel.y >= bounding_box.y || current_voxel.z >= bounding_box.z)
				{
					if(input_lclick_down && i == width_pixels / 2 && j == height_pixels / 2)
						cout << jumps << endl;
					
					if(render_mode == JUMPS)
					{
						blue_and_green = std::max(0, 255 - (max_depth + jumps) * jump_multiplier / 2);
						current_color = vec4(255, blue_and_green, blue_and_green, 255);
					}
					else if(render_mode == NOT_NULL_JUMPS)
					{
						blue_and_green = std::max(0, 255 - max_depth * jump_multiplier);
						current_color = vec4(blue_and_green, blue_and_green, 255, 255);
					}
					else
						current_color = vec4(250, 250, 250, 255);
						
					set_pixel(i, j, current_color);
					//set_pixel(i, j, get_background_pixel(ray_direction));
					break;
				}

				vec4 voxel = get_matrix_voxel(current_voxel);
				if(voxel.w > 0)
				{
					if(render_mode == DEPTH)
					{
						current_color = vec4(
							std::min(255, std::max(0, (int) (total_dist * 250.0f / BOUNDING_BOX_SIZE))),
							std::min(255, std::max(0, (int) (total_dist * 250.0f / BOUNDING_BOX_SIZE))),
							std::min(255, std::max(0, (int) (total_dist * 250.0f / BOUNDING_BOX_SIZE))),
							255);
					}
					else if(render_mode == COLOR)
					{
						current_color = vec4(
							std::max(0, std::min(255, (int) (voxel.x * (2.0f / (1 + exp(total_dist / 64.0f)))))),
							std::max(0, std::min(255, (int) (voxel.y * (2.0f / (1 + exp(total_dist / 64.0f)))))),
							std::max(0, std::min(255, (int) (voxel.z * (2.0f / (1 + exp(total_dist / 64.0f)))))),
							255);
						face_normal = -voxel_incr * step;
						//current_color *= float(dot(-face_normal, directional_light) * 0.25 + 0.75);
						current_color.w = 255;
					}
					else if(render_mode == JUMPS)
					{
						blue_and_green = std::max(0, 255 - (max_depth + jumps) * jump_multiplier / 2);
						current_color = vec4(255, blue_and_green, blue_and_green, 255);
					}
					else if(render_mode == NOT_NULL_JUMPS)
					{
						blue_and_green = std::max(0, 255 - max_depth * jump_multiplier);
						current_color = vec4(blue_and_green, blue_and_green, 255, 255);
					}
					else
						current_color = vec4(250, 250, 250, 255);
					
					set_pixel(i, j, current_color);
					break;
				}
			}
		}
	}
}

void octree_set_default_stack(vec3 point, float &node_size, vec3 &proportional_position)
{
	stack_node_t current_item = default_stack.top();

	int depth = default_stack.size() - 1;
	float size = octree->size / exp2(depth);

	vec3 position = current_item.position;

	node_size = size;
	proportional_position.x = clamp(point.x / size - position.x, 0.0f, 1.0f);
	proportional_position.y = clamp(point.y / size - position.y, 0.0f, 1.0f);
	proportional_position.z = clamp(point.z / size - position.z, 0.0f, 1.0f);

	if(current_item.node == nullptr || depth == octree->depth || current_item.node->leaf)
		return;

	vec3 child_position = vec3(
		point.x >= size * position.x + size * 0.5f,
		point.y >= size * position.y + size * 0.5f,
		point.z >= size * position.z + size * 0.5f
		);

	int child_index = (int(child_position.x) << 0) | (int(child_position.y) << 1) | (int(child_position.z) << 2);

	position = vec3(
		int(position.x) << 1 | int(child_position.x),
		int(position.y) << 1 | int(child_position.y),
		int(position.z) << 1 | int(child_position.z)
		);

	stack_node_t new_item;
	new_item.node = current_item.node->children[child_index];
	new_item.position = position;
	default_stack.push(new_item);

	octree_set_default_stack(point, node_size, proportional_position);
}

void get_octree_node_size(node_t** node, vec3 point, vec3 real_point, vec3 position, int depth, float* node_size_ptr, vec3* proportional_position_ptr)
{
	float size = octree->size / exp2(depth);

	*node_size_ptr = size;
	proportional_position_ptr->x = clamp(real_point.x / size - position.x, 0.0f, 1.0f);
	proportional_position_ptr->y = clamp(real_point.y / size - position.y, 0.0f, 1.0f);
	proportional_position_ptr->z = clamp(real_point.z / size - position.z, 0.0f, 1.0f);

	if(*node == nullptr || (*node)->leaf)
		return;

	vec3 child_position = vec3(
		point.x >= size * position.x + size * 0.5f,
		point.y >= size * position.y + size * 0.5f,
		point.z >= size * position.z + size * 0.5f
		);

	int child_index = (int(child_position.x) << 0) | (int(child_position.y) << 1) | (int(child_position.z) << 2);

	position = vec3(
		int(position.x) << 1 | int(child_position.x),
		int(position.y) << 1 | int(child_position.y),
		int(position.z) << 1 | int(child_position.z)
		);

	get_octree_node_size(&(*node)->children[child_index], point, real_point, position, ++depth, node_size_ptr, proportional_position_ptr);
}

bool octree_get(node_t** node, vec3 point, vec3 real_point, vec3 position, int depth, vec4 &node_color, float &node_size, vec3 &proportional_position)
{
	float size = octree->size / exp2(depth);

	node_size = size;
	proportional_position.x = clamp(real_point.x / size - position.x, 0.0f, 1.0f);
	proportional_position.y = clamp(real_point.y / size - position.y, 0.0f, 1.0f);
	proportional_position.z = clamp(real_point.z / size - position.z, 0.0f, 1.0f);

	if(*node == nullptr)
		return false;

	node_color = (*node)->voxel_color;

	if(depth == octree->depth || (*node)->leaf)
		return true;

	vec3 child_position = vec3(
		point.x >= size * position.x + size * 0.5f,
		point.y >= size * position.y + size * 0.5f,
		point.z >= size * position.z + size * 0.5f
		);

	int child_index = (int(child_position.x) << 0) | (int(child_position.y) << 1) | (int(child_position.z) << 2);

	position = vec3(
		int(position.x) << 1 | int(child_position.x),
		int(position.y) << 1 | int(child_position.y),
		int(position.z) << 1 | int(child_position.z)
		);

	return octree_get(&(*node)->children[child_index], point, real_point, position, depth + 1, node_color, node_size, proportional_position);
}

int octree_get_stack(vec3 point, vec3 real_point, vec4 &node_color, float &node_size, vec3 &proportional_position, bool &solid_voxel)
{
	stack_node_t stack_item = node_stack.top();
	node_t* node = stack_item.node;

	int depth = node_stack.size() - 1;
	float size = octree->size / exp2(depth);

	if(depth > max_depth)
		max_depth = depth;

	vec3 position = stack_item.position;

	node_size = size;
	proportional_position.x = real_point.x / size - position.x;
	proportional_position.y = real_point.y / size - position.y;
	proportional_position.z = real_point.z / size - position.z;

	bool not_inside = 	point.x < position.x * size || point.x > (position.x + 1) * size ||
						point.y < position.y * size || point.y > (position.y + 1) * size ||
						point.z < position.z * size || point.z > (position.z + 1) * size;

	if(not_inside)
	{
		node_stack.pop();
		return 0;
	}

	if(node == nullptr)
	{
		solid_voxel = false;
		return 2;
	}

	if(depth == octree->depth || node->leaf)		// la comparación leaf es redundante si no se abrevia la búsqueda comparando que todos sus hijos sean iguales
	{
		node_color = node->voxel_color;		// debería estar antes del if si se asignasen colores a los nodos padres para usar LOD
		solid_voxel = true;
		max_depth++;
		return 2;
	}

	vec3 child_position = vec3(
		point.x >= size * position.x + size * 0.5f,
		point.y >= size * position.y + size * 0.5f,
		point.z >= size * position.z + size * 0.5f
		);

	int child_index = (int(child_position.x) << 0) | (int(child_position.y) << 1) | (int(child_position.z) << 2);

	position = vec3(
		int(position.x) << 1 | int(child_position.x),
		int(position.y) << 1 | int(child_position.y),
		int(position.z) << 1 | int(child_position.z)
		);

	stack_node_t item;
	item.node = node->children[child_index];
	item.position = position;
	node_stack.push(item);
	return 1;
}

bool get_octree_voxel(vec3 v, vec3 rv, vec4 &voxel_color, float &node_size, vec3 &proportional_position)
{
	return octree_get(&octree->root, v, rv, vec3(), 0, voxel_color, node_size, proportional_position);
}

bool get_octree_voxel_stack(vec3 v, vec3 rv, vec4 &voxel_color, float &node_size, vec3 &proportional_position)
{
	// octree_get_stack retorna
	// 0 si hace pop() y se debe subir en el octree
	// 1 si hace push() de un nodo nuevo y hay que bajar
	// 2 si encuentra un nodo hoja y se debe terminar la búsqueda
	// técnicamente nunca hará pop() del nodo root.
	bool solid_voxel;
	int it = 0;
	do
	{
		if(node_stack.empty())
		{
			stack_node_t root_item;
			root_item.node = octree->root;
			root_item.position = vec3();
			node_stack.push(root_item);
		}
		if(it++ > 20)
		{
			cout << "stack loop" << endl;
		}
	} while(octree_get_stack(v, rv, voxel_color, node_size, proportional_position, solid_voxel) < 2);

	return solid_voxel;
}

void octree_insert(node_t** node, vec3 point, vec4 color, vec3 position, int depth)
{
	if(*node == nullptr)
	{
		*node = (node_t*) malloc(sizeof(node_t));
		for (int i = 0; i < 8; ++i)
			(*node)->children[i] = nullptr;
	}

	(*node)->voxel_color = color;
	if(depth == octree->depth)
	{
		(*node)->leaf = true;
		return;
	}

	float size = octree->size / exp2(depth);

	vec3 child_position = vec3(
		point.x >= size * position.x + size * 0.5f,
		point.y >= size * position.y + size * 0.5f,
		point.z >= size * position.z + size * 0.5f
		);

	int child_index = (int(child_position.x) << 0) | (int(child_position.y) << 1) | (int(child_position.z) << 2);

	position = vec3(
		int(position.x) << 1 | int(child_position.x),
		int(position.y) << 1 | int(child_position.y),
		int(position.z) << 1 | int(child_position.z)
		);

	octree_insert(&(*node)->children[child_index], point, color, position, ++depth);

	// recursive checking if childrens are the same so the parent can become leaf and destroy all children
	//for(int i = 0; i < )
}

void raycast_octree()
{
	for (int i = 0; i < width_pixels; ++i)
	{
		for (int j = 0; j < height_pixels; ++j)
		{
			vec3 right_offset = vec3(camera_right);
			right_offset *= ((i - width_pixels / 2.0) / height_pixels) * raycast_offset_multiplier;

			vec3 up_offset = vec3(camera_up);
			up_offset *= ((j - height_pixels / 2.0) / height_pixels) * raycast_offset_multiplier;

			vec3 ray_direction = normalize(vec3(camera_forward + right_offset + up_offset));

			float total_dist = 0;

			vec3 ray_hit = camera_pos;
			vec3 ray_hit_offseted = camera_pos;

			vec3 face_normal = vec3();

			float node_size;
			vec3 proportional_position;

			if(outside)
			{
				if(ray_intersects_aabb(camera_pos, ray_direction, bounding_box, &ray_hit, &face_normal))
				{
					ray_hit_offseted = ray_hit - face_normal * 0.5f;
					total_dist = distance(camera_pos, ray_hit);
					vec4 voxel_color;

					if(get_octree_voxel(ray_hit_offseted, ray_hit, voxel_color, node_size, proportional_position))
					{
						if(render_mode == DEPTH)
						{
							current_color = vec4(
								std::min(255, std::max(0, (int) (total_dist * 100.0f / BOUNDING_BOX_SIZE))),
								std::min(255, std::max(0, (int) (total_dist * 100.0f / BOUNDING_BOX_SIZE))),
								std::min(255, std::max(0, (int) (total_dist * 100.0f / BOUNDING_BOX_SIZE))),
								255);
						}
						else if(render_mode == COLOR)
						{
							current_color = voxel_color;
							//current_color *= float(dot(-face_normal, directional_light) * 0.25 + 0.75);
							current_color.w = 255;
						}
						set_pixel(i, j, current_color);
						continue;
					}
				}
				else
				{
					set_pixel(i, j, vec4(255, 255, 255, 255));
					continue;
				}
			}

			// traversal
			vec3 step = vec3(1, 1, 1);						// Signo de cada componente del vector ray_direction
			vec3 deltaT = vec3(1, 1, 1) * 999999.0f;		// Distancia que cada componente avanzaría si el rayo recorriese 1 unidad en ese axis
			vec3 distance = vec3(1, 1, 1) * 999999.0f;		// Distancia que debe recorrer el rayo para chocar con la siguiente pared de ese axis

			step.x = sgn(ray_direction.x);
			if(abs(ray_direction.x) > 0.00001f)
				deltaT.x = abs(1.0f / ray_direction.x);

			step.y = sgn(ray_direction.y);
			if(abs(ray_direction.y) > 0.00001f)
				deltaT.y = abs(1.0f / ray_direction.y);

			step.z = sgn(ray_direction.z);
			if(abs(ray_direction.z) > 0.00001f)
				deltaT.z = abs(1.0f / ray_direction.z);

			vec3 voxel_incr = vec3(0, 0, 0);	// Máscara de axis de la colisión actual, si el rayo acaba de entrar
												// a un voxel por alguna de sus caras con normal x, entonces voxel_incr
												// será vec3(1, 0, 0).

												// Se multiplicará con step, (vector que guarda el signo de cada componente) para tener el vector de desplazamiento

			if(!outside)
			{
				node_size = start_node_size;
				proportional_position = start_proportional_position;
			}

			int it = 0;
			while(it++ < 3 * BOUNDING_BOX_SIZE)
			{
				if((int) voxel_incr.x != 0)
					distance.x = node_size * deltaT.x;
				else if(step.x > 0)
					distance.x = (1.0f - proportional_position.x) * node_size * deltaT.x;
				else if(step.x < 0)
					distance.x = proportional_position.x * node_size * deltaT.x;
				else
					distance.x = 9999;

				if((int) voxel_incr.y != 0)
					distance.y = node_size * deltaT.y;
				else if(step.y > 0)
					distance.y = (1.0f - proportional_position.y) * node_size * deltaT.y;
				else if(step.y < 0)
					distance.y = proportional_position.y * node_size * deltaT.y;
				else
					distance.y = 9999;

				if((int) voxel_incr.z != 0)
					distance.z = node_size * deltaT.z;
				else if(step.z > 0)
					distance.z = (1.0f - proportional_position.z) * node_size * deltaT.z;
				else if(step.z < 0)
					distance.z = proportional_position.z * node_size * deltaT.z;
				else
					distance.z = 9999;

				float min_distance = 9999;
				if(distance.x < min_distance)
					min_distance = distance.x;
				if(distance.y < min_distance)
					min_distance = distance.y;
				if(distance.z < min_distance)
					min_distance = distance.z;

				voxel_incr = vec3(0, 0, 0);
				if((distance.x <= distance.y) && (distance.x <= distance.z))
					voxel_incr.x = 1;
				else if((distance.y <= distance.x) && (distance.y <= distance.z))
					voxel_incr.y = 1;
				else if((distance.z <= distance.x) && (distance.z <= distance.y))
					voxel_incr.z = 1;

				total_dist += min_distance;
				ray_hit = camera_pos + ray_direction * total_dist;
				ray_hit_offseted = ray_hit + voxel_incr * step * 0.5f;

				if(ray_hit_offseted.x < 0 || ray_hit_offseted.y < 0 || ray_hit_offseted.z < 0 ||
				ray_hit_offseted.x > bounding_box.x || ray_hit_offseted.y > bounding_box.y || ray_hit_offseted.z > bounding_box.z)
				{
					set_pixel(i, j, vec4(255, 255, 255, 255));
					//set_pixel(i, j, get_background_pixel(ray_direction));
					break;
				}

				vec4 voxel_color;
				if(get_octree_voxel(ray_hit_offseted, ray_hit, voxel_color, node_size, proportional_position))
				{
					if(render_mode == DEPTH)
					{
						current_color = vec4(
							std::min(255, std::max(0, (int) (total_dist * 100.0f / BOUNDING_BOX_SIZE))),
							std::min(255, std::max(0, (int) (total_dist * 100.0f / BOUNDING_BOX_SIZE))),
							std::min(255, std::max(0, (int) (total_dist * 100.0f / BOUNDING_BOX_SIZE))),
							255);
					}
					else if(render_mode == COLOR)
					{
						current_color = voxel_color;
						face_normal = - voxel_incr * step;
						//current_color *= float(dot(-face_normal, directional_light) * 0.25 + 0.75);
						current_color.w = 255;
					}
					set_pixel(i, j, current_color);
					break;
				}
			}
		}
	}
}

void raycast_octree_stack()
{
	for (int i = 0; i < width_pixels; ++i)
	{
		for (int j = 0; j < height_pixels; ++j)
		{
			jumps = 0;
			not_null_jumps = 0;
			max_depth = 0;

			vec3 right_offset = vec3(camera_right);
			right_offset *= ((i - width_pixels * 0.5f) / height_pixels) * raycast_offset_multiplier;

			vec3 up_offset = vec3(camera_up);
			up_offset *= ((j - height_pixels * 0.5f) / height_pixels) * raycast_offset_multiplier;

			vec3 ray_direction = normalize(vec3(camera_forward + right_offset + up_offset));

			float total_dist = 0;

			vec3 ray_hit = camera_pos;
			vec3 ray_hit_offseted = camera_pos;

			vec3 face_normal = vec3();

			float node_size;
			vec3 proportional_position;

			if(outside)
			{
				if(ray_intersects_aabb(camera_pos, ray_direction, bounding_box, &ray_hit, &face_normal))
				{
					ray_hit_offseted = ray_hit - face_normal * 0.5f;
					ray_hit_offseted.x = clamp(ray_hit_offseted.x, 0.0f, bounding_box.x);
					ray_hit_offseted.y = clamp(ray_hit_offseted.y, 0.0f, bounding_box.y);
					ray_hit_offseted.z = clamp(ray_hit_offseted.z, 0.0f, bounding_box.z);
					total_dist = distance(camera_pos, ray_hit);
					vec4 voxel_color;

					if(get_octree_voxel_stack(ray_hit_offseted, ray_hit, voxel_color, node_size, proportional_position))
					{
						if(render_mode == DEPTH)
						{
							current_color = vec4(
								std::min(255, std::max(0, (int) (total_dist * 100.0f / BOUNDING_BOX_SIZE))),
								std::min(255, std::max(0, (int) (total_dist * 100.0f / BOUNDING_BOX_SIZE))),
								std::min(255, std::max(0, (int) (total_dist * 100.0f / BOUNDING_BOX_SIZE))),
								255);
						}
						else if(render_mode == COLOR)
						{
							current_color = voxel_color;
							//current_color *= float(dot(-face_normal, directional_light) * 0.25 + 0.75);
							current_color.w = 255;
						}
						else if(render_mode == JUMPS)
						{
							blue_and_green = std::max(0, 255 - (max_depth + jumps) * jump_multiplier * 2);
							current_color = vec4(255, blue_and_green, blue_and_green, 255);
						}
						else if(render_mode == NOT_NULL_JUMPS)
						{
							blue_and_green = std::max(0, 255 - max_depth * jump_multiplier * 4);
							current_color = vec4(blue_and_green, blue_and_green, 255, 255);
						}

						set_pixel(i, j, current_color);
						continue;
					}
					jumps++;
				}
				else
				{
					set_pixel(i, j, vec4(255, 255, 255, 255));
					continue;
				}
			}
			else
			{
				node_stack = default_stack;
				node_size = start_node_size;
				proportional_position = start_proportional_position;
			}

			// traversal
			vec3 step = vec3(1, 1, 1);						// Signo de cada componente del vector ray_direction
			vec3 deltaT = vec3(1, 1, 1) * 999999.0f;		// Distancia que cada componente avanzaría si el rayo recorriese 1 unidad en ese axis
			vec3 distance = vec3(1, 1, 1) * 999999.0f;		// Distancia que debe recorrer el rayo para chocar con la siguiente pared de ese axis

			step.x = sgn(ray_direction.x);
			if(abs(ray_direction.x) > 0.00001f)
				deltaT.x = abs(1.0f / ray_direction.x);

			step.y = sgn(ray_direction.y);
			if(abs(ray_direction.y) > 0.00001f)
				deltaT.y = abs(1.0f / ray_direction.y);

			step.z = sgn(ray_direction.z);
			if(abs(ray_direction.z) > 0.00001f)
				deltaT.z = abs(1.0f / ray_direction.z);

			vec3 voxel_incr = vec3(0, 0, 0);	// Máscara de axis de la colisión actual, si el rayo acaba de entrar
												// a un voxel por alguna de sus caras con normal x, entonces voxel_incr
												// será vec3(1, 0, 0).

												// Se multiplicará con step, (vector que guarda el signo de cada componente) para tener el vector de desplazamiento

			int it = 0;
			while(it++ < 3 * BOUNDING_BOX_SIZE)
			{
				if((int) voxel_incr.x != 0)
					distance.x = node_size * deltaT.x;
				else if(step.x > 0)
					distance.x = (1.0f - proportional_position.x) * node_size * deltaT.x;
				else if(step.x < 0)
					distance.x = proportional_position.x * node_size * deltaT.x;
				else
					distance.x = 9999;

				if((int) voxel_incr.y != 0)
					distance.y = node_size * deltaT.y;
				else if(step.y > 0)
					distance.y = (1.0f - proportional_position.y) * node_size * deltaT.y;
				else if(step.y < 0)
					distance.y = proportional_position.y * node_size * deltaT.y;
				else
					distance.y = 9999;

				if((int) voxel_incr.z != 0)
					distance.z = node_size * deltaT.z;
				else if(step.z > 0)
					distance.z = (1.0f - proportional_position.z) * node_size * deltaT.z;
				else if(step.z < 0)
					distance.z = proportional_position.z * node_size * deltaT.z;
				else
					distance.z = 9999;

				float min_distance = 9999;
				if(distance.x < min_distance)
					min_distance = distance.x;
				if(distance.y < min_distance)
					min_distance = distance.y;
				if(distance.z < min_distance)
					min_distance = distance.z;

				voxel_incr = vec3(0, 0, 0);
				if((distance.x <= distance.y) && (distance.x <= distance.z))
					voxel_incr.x = step.x;
				else if((distance.y <= distance.x) && (distance.y <= distance.z))
					voxel_incr.y = step.y;
				else if((distance.z <= distance.x) && (distance.z <= distance.y))
					voxel_incr.z = step.z;

				total_dist += min_distance;
				ray_hit = camera_pos + ray_direction * total_dist;
				ray_hit_offseted = ray_hit + voxel_incr * 0.5f;
				jumps++;

				if(ray_hit_offseted.x < 0 || ray_hit_offseted.y < 0 || ray_hit_offseted.z < 0 ||
				ray_hit_offseted.x > bounding_box.x || ray_hit_offseted.y > bounding_box.y || ray_hit_offseted.z > bounding_box.z)
				{
					if(input_lclick_down && i == width_pixels / 2 && j == height_pixels / 2)
						cout << jumps << endl;
					
					if(render_mode == JUMPS)
					{
						blue_and_green = std::max(0, 255 - (max_depth + jumps) * jump_multiplier / 2);
						current_color = vec4(255, blue_and_green, blue_and_green, 255);
					}
					else if(render_mode == NOT_NULL_JUMPS)
					{
						blue_and_green = std::max(0, 255 - max_depth * jump_multiplier);
						current_color = vec4(blue_and_green, blue_and_green, 255, 255);
					}
					else
						current_color = vec4(250, 250, 250, 255);

					set_pixel(i, j, current_color);
					break;
				}

				vec4 voxel_color;
				if(get_octree_voxel_stack(ray_hit_offseted, ray_hit, voxel_color, node_size, proportional_position))
				{
					if(input_lclick_down && i == width_pixels / 2 && j == height_pixels / 2)
						cout << jumps << endl;
					
					if(render_mode == DEPTH)
					{
						current_color = vec4(
							std::min(255, std::max(0, (int) (total_dist * 100.0f / BOUNDING_BOX_SIZE))),
							std::min(255, std::max(0, (int) (total_dist * 100.0f / BOUNDING_BOX_SIZE))),
							std::min(255, std::max(0, (int) (total_dist * 100.0f / BOUNDING_BOX_SIZE))),
							255);
					}
					else if(render_mode == COLOR)
					{
						current_color = voxel_color;
						face_normal = - voxel_incr;
						//current_color *= float(dot(-face_normal, directional_light) * 0.25 + 0.75);
						current_color.w = 255;
					}
					else if(render_mode == JUMPS)
					{
						blue_and_green = std::max(0, 255 - (max_depth + jumps) * jump_multiplier * 2);
						current_color = vec4(255, blue_and_green, blue_and_green, 255);
					}
					else if(render_mode == NOT_NULL_JUMPS)
					{
						blue_and_green = std::max(0, 255 - max_depth * jump_multiplier * 4);
						current_color = vec4(blue_and_green, blue_and_green, 255, 255);
					}

					set_pixel(i, j, current_color);
					break;
				}
			}
		}
	}
}

void draw()
{
	outside = camera_pos.x < 0 || camera_pos.x > bounding_box.x ||
		camera_pos.y < 0 || camera_pos.y > bounding_box.y ||
		camera_pos.z < 0 || camera_pos.z > bounding_box.z;
		
	if(!outside)
	{
		if(data_type == OCTREE)
			//octree_set_default_stack(camera_pos, start_node_size, start_proportional_position);
			get_octree_node_size(&octree->root, camera_pos, camera_pos, vec3(), 0, &start_node_size, &start_proportional_position);
	}

	// write the pixels
	SDL_LockSurface(pixels);

	if(data_type == MATRIX)
		raycast_matrix();
	else if(data_type == OCTREE)
	{
		//raycast_octree();
		raycast_octree_stack();
	}
	else if(data_type == KDTREE)
	{
		//
	}

	set_pixel(width_pixels / 2 - 2, height_pixels / 2, vec4(255, 255, 255, 255));
	set_pixel(width_pixels / 2 + 2, height_pixels / 2, vec4(255, 255, 255, 255));
	set_pixel(width_pixels / 2, height_pixels / 2 - 2, vec4(255, 255, 255, 255));
	set_pixel(width_pixels / 2, height_pixels / 2 + 2, vec4(255, 255, 255, 255));
	set_pixel(width_pixels / 2 - 3, height_pixels / 2, vec4(255, 255, 255, 255));
	set_pixel(width_pixels / 2 + 3, height_pixels / 2, vec4(255, 255, 255, 255));
	set_pixel(width_pixels / 2, height_pixels / 2 - 3, vec4(255, 255, 255, 255));
	set_pixel(width_pixels / 2, height_pixels / 2 + 3, vec4(255, 255, 255, 255));
	set_pixel(width_pixels / 2 - 4, height_pixels / 2, vec4(255, 255, 255, 255));
	set_pixel(width_pixels / 2 + 4, height_pixels / 2, vec4(255, 255, 255, 255));
	set_pixel(width_pixels / 2, height_pixels / 2 - 4, vec4(255, 255, 255, 255));
	set_pixel(width_pixels / 2, height_pixels / 2 + 4, vec4(255, 255, 255, 255));
	SDL_UnlockSurface(pixels);

	SDL_BlitScaled(pixels, NULL, screen, &dst_rect);
	SDL_UpdateWindowSurface(window);

	//SDL_RenderPresent(renderer);
}

void load_mesh()
{
	try {
		StlMesh <float, unsigned int> mesh ("gato.stl");

		// MESH MAPPING PART
		// prefix g is for geometry, the values for the bounding box that contains the mesh entirely
		// aabb: axis aligned bounding box

		// this values will help to find the whole geometry's bounding box
		vec3 g_aabb_min = vec3( 9999.0f,  9999.0f,  9999.0f);
		vec3 g_aabb_max = vec3(-9999.0f, -9999.0f, -9999.0f);

		vec3 vertices[3];
		vec3 g_angles = vec3(-90, 0, 0);

		bool valid = true;
		for(size_t itri = 0; itri < mesh.num_tris(); ++itri) {
			for(size_t icorner = 0; icorner < 3; ++icorner) {
				const float* c = mesh.tri_corner_coords (itri, icorner);
				// or alternatively:
				// float* c = mesh.vrt_coords (mesh.tri_corner_ind (itri, icorner));

				if(isnan(c[0]) || isnan(c[1]) || isnan(c[2]))
				{
					valid = false;
					break;
				}

				if(c[0] < g_aabb_min.x) g_aabb_min.x = c[0];
				if(c[1] < g_aabb_min.y) g_aabb_min.y = c[1];
				if(c[2] < g_aabb_min.z) g_aabb_min.z = c[2];

				if(c[0] > g_aabb_max.x) g_aabb_max.x = c[0];
				if(c[1] > g_aabb_max.y) g_aabb_max.y = c[1];
				if(c[2] > g_aabb_max.z) g_aabb_max.z = c[2];
			}
		}

		vec3 g_aabb_size = vec3(g_aabb_max.x - g_aabb_min.x, g_aabb_max.y - g_aabb_min.y, g_aabb_max.z - g_aabb_min.z);
		float max_comp = ((g_aabb_size.x > g_aabb_size.y) ? ((g_aabb_size.x > g_aabb_size.z) ? g_aabb_size.x : g_aabb_size.z) : ((g_aabb_size.y > g_aabb_size.z) ? g_aabb_size.y : g_aabb_size.z)) + 0.0001f;
		float g_scale = BOUNDING_BOX_SIZE / max_comp;

		vec3 g_aabb_center = vec3((g_aabb_max.x + g_aabb_min.x) / 2.0f, (g_aabb_max.y + g_aabb_min.y) / 2.0f, (g_aabb_max.z + g_aabb_min.z) / 2.0f);
		g_aabb_min = vec3(g_aabb_center.x - max_comp / 2.0f, g_aabb_center.y - max_comp / 2.0f, g_aabb_center.z - max_comp / 2.0f);
		g_aabb_max = vec3(g_aabb_center.x + max_comp / 2.0f, g_aabb_center.y + max_comp / 2.0f, g_aabb_center.z + max_comp / 2.0f);

		// TRIANGLE AABB PART
		// prefix t is for triangle

		// values for the bounding box that contains the current triangle
		vec3 t_aabb_min;
		vec3 t_aabb_max;

		valid = true;
		for(int itri = 0; itri < mesh.num_tris(); itri++)
		{
			// this values will help to find the triangle's bounding box
			vec3 t_aabb_min = bounding_box + vec3(9991, 9991, 9991);	// 1 unit beyond the limits of the root aabb, since the vertex was mapped already it is inside the root aabb
			vec3 t_aabb_max = vec3(-9991, -9991, -9991); // 1 unit behind the origin of the root aabb, which is (0, 0, 0)

			for(int icorner = 0; icorner < 3; icorner++)
			{
				const float* c = mesh.tri_corner_coords (itri, icorner);
				if(isnan(c[0]) || isnan(c[1]) || isnan(c[2]))
				{
					valid = false;
					break;
				}

				// this transformation picks the vertex of the mesh and expands it to ocupy the whole root aabb
				vertices[icorner] = vec3(c[0], c[1], c[2]);
				vertices[icorner] -= g_aabb_min;
				vertices[icorner] *= g_scale;
				vertices[icorner] -= bounding_box * 0.5f;
				vertices[icorner] = rotateZ(vertices[icorner], radians(g_angles.z));
				vertices[icorner] = rotateX(vertices[icorner], radians(g_angles.x));
				vertices[icorner] = rotateY(vertices[icorner], radians(g_angles.y));
				vertices[icorner] += bounding_box * 0.5f;

				// finding the highest and lowest components to set the aabb
				if(vertices[icorner].x < t_aabb_min.x) t_aabb_min.x = vertices[icorner].x;
				if(vertices[icorner].y < t_aabb_min.y) t_aabb_min.y = vertices[icorner].y;
				if(vertices[icorner].z < t_aabb_min.z) t_aabb_min.z = vertices[icorner].z;

				if(vertices[icorner].x > t_aabb_max.x) t_aabb_max.x = vertices[icorner].x;
				if(vertices[icorner].y > t_aabb_max.y) t_aabb_max.y = vertices[icorner].y;
				if(vertices[icorner].z > t_aabb_max.z) t_aabb_max.z = vertices[icorner].z;
			}
			if(!valid)
				continue;

			// INTERSECTION PART
			// prefix v is for voxel
			
			// this is the half of each voxel (if voxels are (1,1,1) sized);
			vec3 v_aabb_half = vec3(0.5f, 0.5f, 0.5f);
			vec3 v_aabb_relative_vertex[] = {
				vec3(0.0f, 0.0f, 0.0f),
				vec3(1.0f, 0.0f, 0.0f),
				vec3(0.0f, 1.0f, 0.0f),
				vec3(1.0f, 1.0f, 0.0f),
				vec3(0.0f, 0.0f, 1.0f),
				vec3(1.0f, 0.0f, 1.0f),
				vec3(0.0f, 1.0f, 1.0f),
				vec3(1.0f, 1.0f, 1.0f)
			};

			// flag that indicates the rule for the comparisons of the projected points
			// if false, the comparisons should be all aabb > triangle for each projected point in order to be not colliding
			// equally, if the comparison returns aabb < triangle for at least one projected point, they ARE COLLIDING
			bool lower_than;

			// normal of the current triangle
			const float* n = mesh.tri_normal (itri);
			vec3 t_normal = vec3(n[0], n[1], n[2]);
			t_normal = rotateZ(t_normal, radians(g_angles.z));
			t_normal = rotateX(t_normal, radians(g_angles.x));
			t_normal = rotateY(t_normal, radians(g_angles.y));

			// separating axis test (SAP) between every voxel inside triangle aabb with this current triangle
			for (unsigned int i = t_aabb_min.x; i < t_aabb_max.x; ++i)
				for (unsigned int j = t_aabb_min.y; j < t_aabb_max.y; ++j)
					for (unsigned int k = t_aabb_min.z; k < t_aabb_max.z; ++k)
					{
						// is set initially to the comparison rule of the first voxel
						lower_than = false;

						float v_min_proj = 9999.0f;
						float v_max_proj = -9999.0f;
						float t_min_proj = 9999.0f;
						float t_max_proj = -9999.0f;

						vec3 axis = vec3(1, 0, 0);

						for (int icorner = 0; icorner < 3; ++icorner)
						{
							vec3 proj_vec = proj(vertices[icorner], axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < t_min_proj) t_min_proj = proj_length;
							if(proj_length > t_max_proj) t_max_proj = proj_length;
						}
						for (int ivertex = 0; ivertex < 8; ++ivertex)
						{
							vec3 proj_vec = proj(v_aabb_relative_vertex[ivertex] + vec3(i, j, k), axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < v_min_proj) v_min_proj = proj_length;
							if(proj_length > v_max_proj) v_max_proj = proj_length;
						}

						if((t_min_proj < v_min_proj && t_min_proj < v_max_proj && t_max_proj < v_min_proj && t_max_proj < v_max_proj) ||
							(t_min_proj > v_min_proj && t_min_proj > v_max_proj && t_max_proj > v_min_proj && t_max_proj > v_max_proj))
							continue;

						v_min_proj = 9999.0f;
						v_max_proj = -9999.0f;
						t_min_proj = 9999.0f;
						t_max_proj = -9999.0f;
						axis = vec3(0, 1, 0);

						for (int icorner = 0; icorner < 3; ++icorner)
						{
							vec3 proj_vec = proj(vertices[icorner], axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < t_min_proj) t_min_proj = proj_length;
							if(proj_length > t_max_proj) t_max_proj = proj_length;
						}
						for (int ivertex = 0; ivertex < 8; ++ivertex)
						{
							vec3 proj_vec = proj(v_aabb_relative_vertex[ivertex] + vec3(i, j, k), axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < v_min_proj) v_min_proj = proj_length;
							if(proj_length > v_max_proj) v_max_proj = proj_length;
						}

						if((t_min_proj < v_min_proj && t_min_proj < v_max_proj && t_max_proj < v_min_proj && t_max_proj < v_max_proj) ||
							(t_min_proj > v_min_proj && t_min_proj > v_max_proj && t_max_proj > v_min_proj && t_max_proj > v_max_proj))
							continue;

						v_min_proj = 9999.0f;
						v_max_proj = -9999.0f;
						t_min_proj = 9999.0f;
						t_max_proj = -9999.0f;
						axis = vec3(0, 0, 1);

						for (int icorner = 0; icorner < 3; ++icorner)
						{
							vec3 proj_vec = proj(vertices[icorner], axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < t_min_proj) t_min_proj = proj_length;
							if(proj_length > t_max_proj) t_max_proj = proj_length;
						}
						for (int ivertex = 0; ivertex < 8; ++ivertex)
						{
							vec3 proj_vec = proj(v_aabb_relative_vertex[ivertex] + vec3(i, j, k), axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < v_min_proj) v_min_proj = proj_length;
							if(proj_length > v_max_proj) v_max_proj = proj_length;
						}

						if((t_min_proj < v_min_proj && t_min_proj < v_max_proj && t_max_proj < v_min_proj && t_max_proj < v_max_proj) ||
							(t_min_proj > v_min_proj && t_min_proj > v_max_proj && t_max_proj > v_min_proj && t_max_proj > v_max_proj))
							continue;

						v_min_proj = 9999.0f;
						v_max_proj = -9999.0f;
						t_min_proj = 9999.0f;
						t_max_proj = -9999.0f;
						axis = t_normal;

						for (int icorner = 0; icorner < 3; ++icorner)
						{
							vec3 proj_vec = proj(vertices[icorner], axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < t_min_proj) t_min_proj = proj_length;
							if(proj_length > t_max_proj) t_max_proj = proj_length;
						}
						for (int ivertex = 0; ivertex < 8; ++ivertex)
						{
							vec3 proj_vec = proj(v_aabb_relative_vertex[ivertex] + vec3(i, j, k), axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < v_min_proj) v_min_proj = proj_length;
							if(proj_length > v_max_proj) v_max_proj = proj_length;
						}

						if((t_min_proj < v_min_proj && t_min_proj < v_max_proj && t_max_proj < v_min_proj && t_max_proj < v_max_proj) ||
							(t_min_proj > v_min_proj && t_min_proj > v_max_proj && t_max_proj > v_min_proj && t_max_proj > v_max_proj))
							continue;

						v_min_proj = 9999.0f;
						v_max_proj = -9999.0f;
						t_min_proj = 9999.0f;
						t_max_proj = -9999.0f;
						axis = cross(vec3(1, 0, 0), normalize(vertices[0] - vertices[1]));

						for (int icorner = 0; icorner < 3; ++icorner)
						{
							vec3 proj_vec = proj(vertices[icorner], axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < t_min_proj) t_min_proj = proj_length;
							if(proj_length > t_max_proj) t_max_proj = proj_length;
						}
						for (int ivertex = 0; ivertex < 8; ++ivertex)
						{
							vec3 proj_vec = proj(v_aabb_relative_vertex[ivertex] + vec3(i, j, k), axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < v_min_proj) v_min_proj = proj_length;
							if(proj_length > v_max_proj) v_max_proj = proj_length;
						}

						if((t_min_proj < v_min_proj && t_min_proj < v_max_proj && t_max_proj < v_min_proj && t_max_proj < v_max_proj) ||
							(t_min_proj > v_min_proj && t_min_proj > v_max_proj && t_max_proj > v_min_proj && t_max_proj > v_max_proj))
							continue;

						v_min_proj = 9999.0f;
						v_max_proj = -9999.0f;
						t_min_proj = 9999.0f;
						t_max_proj = -9999.0f;
						axis = cross(vec3(1, 0, 0), normalize(vertices[1] - vertices[2]));

						for (int icorner = 0; icorner < 3; ++icorner)
						{
							vec3 proj_vec = proj(vertices[icorner], axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < t_min_proj) t_min_proj = proj_length;
							if(proj_length > t_max_proj) t_max_proj = proj_length;
						}
						for (int ivertex = 0; ivertex < 8; ++ivertex)
						{
							vec3 proj_vec = proj(v_aabb_relative_vertex[ivertex] + vec3(i, j, k), axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < v_min_proj) v_min_proj = proj_length;
							if(proj_length > v_max_proj) v_max_proj = proj_length;
						}

						if((t_min_proj < v_min_proj && t_min_proj < v_max_proj && t_max_proj < v_min_proj && t_max_proj < v_max_proj) ||
							(t_min_proj > v_min_proj && t_min_proj > v_max_proj && t_max_proj > v_min_proj && t_max_proj > v_max_proj))
							continue;

						v_min_proj = 9999.0f;
						v_max_proj = -9999.0f;
						t_min_proj = 9999.0f;
						t_max_proj = -9999.0f;
						axis = cross(vec3(1, 0, 0), normalize(vertices[2] - vertices[0]));

						for (int icorner = 0; icorner < 3; ++icorner)
						{
							vec3 proj_vec = proj(vertices[icorner], axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < t_min_proj) t_min_proj = proj_length;
							if(proj_length > t_max_proj) t_max_proj = proj_length;
						}
						for (int ivertex = 0; ivertex < 8; ++ivertex)
						{
							vec3 proj_vec = proj(v_aabb_relative_vertex[ivertex] + vec3(i, j, k), axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < v_min_proj) v_min_proj = proj_length;
							if(proj_length > v_max_proj) v_max_proj = proj_length;
						}

						if((t_min_proj < v_min_proj && t_min_proj < v_max_proj && t_max_proj < v_min_proj && t_max_proj < v_max_proj) ||
							(t_min_proj > v_min_proj && t_min_proj > v_max_proj && t_max_proj > v_min_proj && t_max_proj > v_max_proj))
							continue;

						v_min_proj = 9999.0f;
						v_max_proj = -9999.0f;
						t_min_proj = 9999.0f;
						t_max_proj = -9999.0f;
						axis = cross(vec3(0, 1, 0), normalize(vertices[0] - vertices[1]));

						for (int icorner = 0; icorner < 3; ++icorner)
						{
							vec3 proj_vec = proj(vertices[icorner], axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < t_min_proj) t_min_proj = proj_length;
							if(proj_length > t_max_proj) t_max_proj = proj_length;
						}
						for (int ivertex = 0; ivertex < 8; ++ivertex)
						{
							vec3 proj_vec = proj(v_aabb_relative_vertex[ivertex] + vec3(i, j, k), axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < v_min_proj) v_min_proj = proj_length;
							if(proj_length > v_max_proj) v_max_proj = proj_length;
						}

						if((t_min_proj < v_min_proj && t_min_proj < v_max_proj && t_max_proj < v_min_proj && t_max_proj < v_max_proj) ||
							(t_min_proj > v_min_proj && t_min_proj > v_max_proj && t_max_proj > v_min_proj && t_max_proj > v_max_proj))
							continue;

						v_min_proj = 9999.0f;
						v_max_proj = -9999.0f;
						t_min_proj = 9999.0f;
						t_max_proj = -9999.0f;
						axis = cross(vec3(0, 1, 0), normalize(vertices[1] - vertices[2]));

						for (int icorner = 0; icorner < 3; ++icorner)
						{
							vec3 proj_vec = proj(vertices[icorner], axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < t_min_proj) t_min_proj = proj_length;
							if(proj_length > t_max_proj) t_max_proj = proj_length;
						}
						for (int ivertex = 0; ivertex < 8; ++ivertex)
						{
							vec3 proj_vec = proj(v_aabb_relative_vertex[ivertex] + vec3(i, j, k), axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < v_min_proj) v_min_proj = proj_length;
							if(proj_length > v_max_proj) v_max_proj = proj_length;
						}

						if((t_min_proj < v_min_proj && t_min_proj < v_max_proj && t_max_proj < v_min_proj && t_max_proj < v_max_proj) ||
							(t_min_proj > v_min_proj && t_min_proj > v_max_proj && t_max_proj > v_min_proj && t_max_proj > v_max_proj))
							continue;

						v_min_proj = 9999.0f;
						v_max_proj = -9999.0f;
						t_min_proj = 9999.0f;
						t_max_proj = -9999.0f;
						axis = cross(vec3(0, 1, 0), normalize(vertices[2] - vertices[0]));

						for (int icorner = 0; icorner < 3; ++icorner)
						{
							vec3 proj_vec = proj(vertices[icorner], axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < t_min_proj) t_min_proj = proj_length;
							if(proj_length > t_max_proj) t_max_proj = proj_length;
						}
						for (int ivertex = 0; ivertex < 8; ++ivertex)
						{
							vec3 proj_vec = proj(v_aabb_relative_vertex[ivertex] + vec3(i, j, k), axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < v_min_proj) v_min_proj = proj_length;
							if(proj_length > v_max_proj) v_max_proj = proj_length;
						}

						if((t_min_proj < v_min_proj && t_min_proj < v_max_proj && t_max_proj < v_min_proj && t_max_proj < v_max_proj) ||
							(t_min_proj > v_min_proj && t_min_proj > v_max_proj && t_max_proj > v_min_proj && t_max_proj > v_max_proj))
							continue;

						v_min_proj = 9999.0f;
						v_max_proj = -9999.0f;
						t_min_proj = 9999.0f;
						t_max_proj = -9999.0f;
						axis = cross(vec3(0, 0, 1), normalize(vertices[0] - vertices[1]));

						for (int icorner = 0; icorner < 3; ++icorner)
						{
							vec3 proj_vec = proj(vertices[icorner], axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < t_min_proj) t_min_proj = proj_length;
							if(proj_length > t_max_proj) t_max_proj = proj_length;
						}
						for (int ivertex = 0; ivertex < 8; ++ivertex)
						{
							vec3 proj_vec = proj(v_aabb_relative_vertex[ivertex] + vec3(i, j, k), axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < v_min_proj) v_min_proj = proj_length;
							if(proj_length > v_max_proj) v_max_proj = proj_length;
						}

						if((t_min_proj < v_min_proj && t_min_proj < v_max_proj && t_max_proj < v_min_proj && t_max_proj < v_max_proj) ||
							(t_min_proj > v_min_proj && t_min_proj > v_max_proj && t_max_proj > v_min_proj && t_max_proj > v_max_proj))
							continue;

						v_min_proj = 9999.0f;
						v_max_proj = -9999.0f;
						t_min_proj = 9999.0f;
						t_max_proj = -9999.0f;
						axis = cross(vec3(0, 0, 1), normalize(vertices[1] - vertices[2]));

						for (int icorner = 0; icorner < 3; ++icorner)
						{
							vec3 proj_vec = proj(vertices[icorner], axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < t_min_proj) t_min_proj = proj_length;
							if(proj_length > t_max_proj) t_max_proj = proj_length;
						}
						for (int ivertex = 0; ivertex < 8; ++ivertex)
						{
							vec3 proj_vec = proj(v_aabb_relative_vertex[ivertex] + vec3(i, j, k), axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < v_min_proj) v_min_proj = proj_length;
							if(proj_length > v_max_proj) v_max_proj = proj_length;
						}

						if((t_min_proj < v_min_proj && t_min_proj < v_max_proj && t_max_proj < v_min_proj && t_max_proj < v_max_proj) ||
							(t_min_proj > v_min_proj && t_min_proj > v_max_proj && t_max_proj > v_min_proj && t_max_proj > v_max_proj))
							continue;

						v_min_proj = 9999.0f;
						v_max_proj = -9999.0f;
						t_min_proj = 9999.0f;
						t_max_proj = -9999.0f;
						axis = cross(vec3(0, 0, 1), normalize(vertices[2] - vertices[0]));

						for (int icorner = 0; icorner < 3; ++icorner)
						{
							vec3 proj_vec = proj(vertices[icorner], axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < t_min_proj) t_min_proj = proj_length;
							if(proj_length > t_max_proj) t_max_proj = proj_length;
						}
						for (int ivertex = 0; ivertex < 8; ++ivertex)
						{
							vec3 proj_vec = proj(v_aabb_relative_vertex[ivertex] + vec3(i, j, k), axis);
							float proj_length = length(proj_vec) * dot(proj_vec, axis);
							if(proj_length < v_min_proj) v_min_proj = proj_length;
							if(proj_length > v_max_proj) v_max_proj = proj_length;
						}

						if((t_min_proj < v_min_proj && t_min_proj < v_max_proj && t_max_proj < v_min_proj && t_max_proj < v_max_proj) ||
							(t_min_proj > v_min_proj && t_min_proj > v_max_proj && t_max_proj > v_min_proj && t_max_proj > v_max_proj))
							continue;

						float hue_noise_value = SimplexNoise::noise(i * 0.01f, j * 0.01f, k * 0.01f);
						int hue = int(hue_noise_value * 127.5f + 127.5f);
						
						if(data_type == MATRIX)
							voxel_data[i * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE + j * BOUNDING_BOX_SIZE + k] = hsv2rgb(vec4(hue, 255, 255, 255));
						else if(data_type == OCTREE)
							octree_insert(
								&octree->root,
								vec3(i, j, k),
								hsv2rgb(vec4(hue, 255, 255, 255)),
								vec3(),
								0
							);
						//else if(data_data_type == KDTREE)
						//	voxel_data[i * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE + j * BOUNDING_BOX_SIZE + k] = hsv2rgb(vec4(hue, 255, 255, 255));
					}
		}
	}
	catch (exception& e) {
		cout << e.what() << endl;
	}
}

void initialize()
{
	bounding_box = vec3(BOUNDING_BOX_SIZE, BOUNDING_BOX_SIZE, BOUNDING_BOX_SIZE);

	if(data_type == MATRIX)
	{
		//voxel_data = new vec4[BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE];
		voxel_data = (vec4*) malloc(BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE * sizeof(vec4));
		for(uint i = 0; i < BOUNDING_BOX_SIZE; i++)
			for(uint j = 0; j < BOUNDING_BOX_SIZE; j++)
				for(uint k = 0; k < BOUNDING_BOX_SIZE; k++)
					voxel_data[i * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE + j * BOUNDING_BOX_SIZE + k] = vec4();
	}
	else if(data_type == OCTREE)
	{
		// Octree
		octree = (octree_t*) malloc(sizeof(octree_t));
		octree->size = BOUNDING_BOX_SIZE;
		octree->depth = round(log2(double(BOUNDING_BOX_SIZE)));
		octree->root = (node_t*) malloc(sizeof(node_t));
		for (int i = 0; i < 8; ++i)
			octree->root->children[i] = nullptr;
		octree->root->leaf = false;

		stack_node_t root_item;
		root_item.node = octree->root;
		root_item.position = vec3();
		default_stack.push(root_item);
	}
	else if(data_type == KDTREE)
	{
		//
	}

	if(mesh_type == NOISE)
	{
		for(uint i = 0; i < BOUNDING_BOX_SIZE; i++)
			for(uint j = 0; j < BOUNDING_BOX_SIZE; j++)
				for(uint k = 0; k < BOUNDING_BOX_SIZE; k++)
				{
					float noise_value = SimplexNoise::noise(i * 0.02f, j * 0.02f, k * 0.02f);
					float hue_noise_value = SimplexNoise::noise(i * 0.01f, j * 0.01f, k * 0.01f);
					if(noise_value < 0.125 && noise_value > -0.125 )
					{
						int hue = int(hue_noise_value * 127.5f + 127.5f);
						if(data_type == MATRIX)
							voxel_data[i * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE + j * BOUNDING_BOX_SIZE + k] = hsv2rgb(vec4(hue, 255, 255, 255));
						else if(data_type == OCTREE) 
							octree_insert(
								&octree->root,
								vec3(i, j, k),
								hsv2rgb(vec4(hue, 255, 255, 255)),
								vec3(),
								0
							);
						else if(data_type == KDTREE)
						{
							//
						}
					}
					else
					{
						if(data_type == MATRIX)
							voxel_data[i * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE + j * BOUNDING_BOX_SIZE + k] = vec4(0, 0, 0, 0);
					}
				}
	}
	else if(mesh_type == MESH)
	{
		load_mesh();
	}
	else if(mesh_type == RANDOM)
	{
		for(int i = 0; i < random_points_count; i++)
			if(data_type == MATRIX)
			{
				int i = rand() % BOUNDING_BOX_SIZE;
				int j = rand() % BOUNDING_BOX_SIZE;
				int k = rand() % BOUNDING_BOX_SIZE;
				voxel_data[i * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE + j * BOUNDING_BOX_SIZE + k] = hsv2rgb(vec4(rand() % 255, 255, 255, 255));
			}
			else if(data_type == OCTREE)
				octree_insert(
					&octree->root,
					vec3(rand() % BOUNDING_BOX_SIZE, rand() % BOUNDING_BOX_SIZE, rand() % BOUNDING_BOX_SIZE),
					hsv2rgb(vec4(rand() % 255, 255, 255, 255)),
					vec3(),
					0
				);
			else if(data_type == KDTREE)
			{
				//
			}
	}
	
	if(!input_recreate)
	{
		camera_pos = vec3(1.5, 1.5, -0.5);
		camera_forward = normalize(vec3(0, 0, -1));
		camera_right = cross(camera_forward, world_up);
		camera_up = cross(camera_right, camera_forward);
		camera_angles = vec2(0, 0);
		redraw();
	}
}

void update()
{
	if(input_position)
	{
		camera_pos = vec3(- 0.5, - 0.5, - 0.5);
		camera_forward = vec3(0, 0, 1);
		camera_right = cross(camera_forward, world_up);
		camera_up = cross(camera_right, camera_forward);
		camera_angles = vec2(0, 0);
		input_position = false;

		redraw();
	}
	
	if(input_recreate)
	{
		initialize();
		input_recreate = false;
	}

	// interact placing voxels or removing voxels
	//if(input_lclick_down)
	//	voxel_data[(int) front_voxel.x][(int) front_voxel.y][(int) front_voxel.z] = 0;
	//else if(input_rclick_down)
	//	voxel_data[(int) (front_voxel.x + adjacent_voxel.x)][(int) (front_voxel.y + adjacent_voxel.y)][(int) (front_voxel.z + adjacent_voxel.z)] = (rand() % 8) + 1;

	if(input_lclick_down)
	{
		//cout << "pos: " << camera_pos.x << " " << camera_pos.y << " " << camera_pos.z << endl;
		//cout << "dir: " << camera_forward.x << " " << camera_forward.y << " " << camera_forward.z << endl;
	}

	if(input_rclick_down)
		render_mode = (render_mode_enum) (((int) render_mode + 1) % 4);

	vec3 add = input_forward ? camera_forward : vec3();
	add -= input_left ? camera_right : vec3();
	add -= input_back ? camera_forward : vec3();
	add += input_right ? camera_right : vec3();
	add += input_up ? camera_up : vec3();
	add -= input_down ? camera_up : vec3();

	camera_pos += add * delta_time * (input_shift ? 32.0f : 8.0f);
	if(length(add) >= 1.0f)
	{
		//outside = camera_pos.x < 0 || camera_pos.x > bounding_box.x ||
		//camera_pos.y < 0 || camera_pos.y > bounding_box.y ||
		//camera_pos.z < 0 || camera_pos.z > bounding_box.z;
		//
		//if(!outside)
		//	octree_set_default_stack(camera_pos, start_node_size, start_proportional_position);

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
			else if(event.key.keysym.scancode == SDL_SCANCODE_R) input_recreate = true;
			else if(event.key.keysym.scancode == SDL_SCANCODE_H) input_position = true;
			else if(event.key.keysym.scancode == SDL_SCANCODE_LSHIFT) input_shift = true;
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
			else if(event.key.keysym.scancode == SDL_SCANCODE_R) input_recreate = false;
			else if(event.key.keysym.scancode == SDL_SCANCODE_H) input_position = false;
			else if(event.key.keysym.scancode == SDL_SCANCODE_LSHIFT) input_shift = false;
		}
		else if(event.type == SDL_MOUSEMOTION)
		{
			camera_angles.x = std::min(89.9f, std::max(-89.9f, camera_angles.x + event.motion.yrel * mouse_sens));
			camera_angles.y -= event.motion.xrel * mouse_sens;
			if(camera_angles.y < -180)
				camera_angles.y += 360;
			if(camera_angles.y >= 180)
				camera_angles.y -= 360;

			camera_forward = rotateX(vec3(0, 0, 1), radians(camera_angles.x));
			camera_forward = rotateY(camera_forward, radians(camera_angles.y));

			camera_right = normalize(cross(camera_forward, world_up));
			camera_up = cross(camera_right, camera_forward);

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
		else if(event.type == SDL_MOUSEWHEEL)
		{
			if(event.wheel.y > 0)
				jump_multiplier = std::min(20, jump_multiplier + 1);
			else if(event.wheel.y < 0)
				jump_multiplier = std::max(1, jump_multiplier - 1);
		}
	}
}

void run()
{
	uint it = 0;
	while(!input_quit)
	{
		catch_input();
		update();

		if(draw_again)
		{
			draw();
			draw_again = false;
		}

		chrono::steady_clock::time_point current_time = chrono::steady_clock::now();
		delta_time = chrono::duration_cast<chrono::duration<float>>(current_time - previous_time).count();
		previous_time = current_time;
		smooth_delta_time = 0.8f * smooth_delta_time + 0.2 * delta_time;

		if(it++ % int(std::max(1.0f, 1.0f / delta_time)) == 0)
			SDL_SetWindowTitle(window, to_string((int) (1.0 / smooth_delta_time)).c_str());
	}
}

void cleanup()
{
	delete[] voxel_data;
}

int main(int argc, char *argv[])
{
	RENDER_SCALE = 0.2f;
	BOUNDING_BOX_SIZE = 16;
	data_type = OCTREE;
	mesh_type = RANDOM;
	random_points_count = 100;	// only used if mesh_type is RANDOM

	width_pixels = WIDTH * RENDER_SCALE;
	height_pixels = HEIGHT * RENDER_SCALE;

	srand(time(NULL));

	if(SDL_Init(SDL_INIT_VIDEO|SDL_INIT_AUDIO) != 0) {
		SDL_Log("Unable to initialize SDL: %s", SDL_GetError());
		return 1;
	}

	SDL_ShowCursor(SDL_DISABLE);
	//SDL_RenderSetScale(renderer, 1.0 / RENDER_SCALE, 1.0 / RENDER_SCALE);
	//SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

	screen = SDL_GetWindowSurface(window);
	pixels = SDL_CreateRGBSurfaceWithFormat(0, width_pixels, height_pixels, 32, SDL_PIXELFORMAT_RGBA32);
	pitch = pixels->pitch;

	SDL_GetClipRect(screen, &dst_rect);
	dst_rect.w /= RENDER_SCALE;
	dst_rect.h /= RENDER_SCALE;

	raycast_offset_multiplier = 2.0f * tan(0.5f * fov * 3.141592f / 180.0f);
	directional_light = normalize(directional_light);

	initialize();
	run();
	cleanup();

	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_FreeSurface(pixels);
	SDL_FreeSurface(screen);
	SDL_Quit();

	int* ptr = (int*) malloc(4);
	free(ptr);

	return 0;
}

//	https://stackoverflow.com/questions/60219223/how-do-i-specify-the-include-path-when-i-build-a-program-in-vscode