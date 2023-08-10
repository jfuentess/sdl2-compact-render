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
#include <bitset>
#include <limits>
#include <chrono>
#include <iomanip>
#include <stack>
#include <fstream>
#include <sstream>

#include "SimplexNoise.h"
#include "stl_reader.h"

#include <k3_tree.hpp>
#include <k3_tree_base.hpp>
#include <LIDAR/k3_tree_Lp1.hpp>

#define WIDTH 1280
#define HEIGHT 720

using namespace std;
using namespace glm;
using namespace stl_reader;
using namespace k3tree;

enum data_type_enum
{
	MATRIX,
	OCTREE,
	OCTREE_STACK,
	KDTREE,
	KDTREE_LIDAR
};

enum mesh_type_enum
{
	NOISE,
	NOISE_SOLID,
	MESH,
	RANDOM
};

enum render_mode_enum
{
	COLOR,
	SHADOW,
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
SDL_Window *window = nullptr;
SDL_Renderer *renderer = nullptr;
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
int tree_height;
unsigned long long memory_usage = 0;

vec4* voxel_data;
octree_t* octree;
k3_tree<>* kdtree;
k3_tree_Lp1<>* kdtree_lidar;

std::vector<k3_tree_base<>::point_type> points;
std::vector<k3_tree_Lp1<>::i_point_LIDAR_type> points_lidar;

data_type_enum data_type;
mesh_type_enum mesh_type;
string mesh_filename;
int random_points_count;	// in case mesh_type is RANDOM
float noise_value_higher, noise_value_lower;

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
int jump_multiplier = 4;
int other_component;
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

string benchmark_name;
string output_filename;

bool playing = false;
bool benchmark_mode;
float lerp_value = 0.0f;
float lerp_speed = 1.0f;
float lerp_speed_multiplier;
vec3 origin_camera_pos;
float origin_theta, origin_phi;
vec3 target_camera_pos;
float target_theta, target_phi;
int current_index;
vector<vector<float>> target_positions;

float mouse_sens = 0.2f;
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
bool input_mclick = false;
bool input_mclick_down = false;
bool input_mclick_down_reset = true;
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

bool input_memory_usage_print = false;
bool input_memory_usage_print_down = false;
bool input_memory_usage_print_down_reset = true;

// colors
vec4 voxel_color;

// testing variables
std::chrono::steady_clock::time_point raycasting_start = std::chrono::steady_clock::now();
float time_to_next_sample;
float time_between_samples = 0.1f;
bool first_frame = true;
int fps[120];
int target_samples = 600;
int current_samples = 0;
int* solid_nodes;
unsigned long long rays = 0;
unsigned long long average_jumps = 0;
unsigned long long total_frames = 0;
unsigned int octree_internal_nodes = 0;

float rx = 0, ry = 0, rz = 0;

bool debugging = false;
int debugging_i = 0;
int debugging_j = 0;

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

void set_pixel(int i, int j, uint color)
{
	unsigned int* row = (unsigned int*) ((char*) pixels->pixels + pitch * (height_pixels - 1 - j));
	row[i] = color;
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
					voxel_color = get_matrix_voxel(ray_origin - face_normal * 0.00001f);
					ray_origin -= face_normal * 0.00001f;
					if(voxel_color.w > 0)
					{
						if(render_mode == DEPTH)
						{
							voxel_color = vec4(
								std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
								std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
								std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
								255);
						}
						else if(render_mode == COLOR)
						{
							voxel_color.w = 255;
						}
						else if(render_mode == SHADOW)
						{
							voxel_color *= float(dot(- face_normal, directional_light) * 0.25 + 0.75);
							voxel_color.w = 255;
						}
						else if(render_mode == JUMPS)
						{
							other_component = std::max(0, 128 - (max_depth + jumps) * jump_multiplier);
							voxel_color = vec4(255, other_component, other_component, 255);
						}
						else if(render_mode == NOT_NULL_JUMPS)
						{
							other_component = std::max(0, 128 - max_depth * jump_multiplier);
							voxel_color = vec4(other_component, other_component, 255, 255);
						}

						set_pixel(i, j, voxel_color);
						continue;
					}
					jumps++;
				}
				else
				{
					set_pixel(i, j, vec4(255, 255, 255, 255));
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
					if(render_mode == JUMPS)
					{
						other_component = std::max(0, 255 - (max_depth + jumps) * jump_multiplier);
						voxel_color = vec4(255, other_component, other_component, 255);
					}
					else if(render_mode == NOT_NULL_JUMPS)
					{
						other_component = std::max(0, 255 - max_depth * jump_multiplier);
						voxel_color = vec4(other_component, other_component, 255, 255);
					}
					else
						voxel_color = vec4(255, 255, 255, 255);
						
					set_pixel(i, j, voxel_color);
					break;
				}

				voxel_color = get_matrix_voxel(current_voxel);
				if(voxel_color.w > 0)
				{
					if(render_mode == DEPTH)
					{
						voxel_color = vec4(
							std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
							std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
							std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
							255);
					}
					else if(render_mode == COLOR)
					{
						voxel_color.w = 255;
					}
					else if(render_mode == SHADOW)
					{
						voxel_color *= float(dot(voxel_incr * step, directional_light) * 0.25 + 0.75);
						voxel_color.w = 255;
					}
					else if(render_mode == JUMPS)
					{
						other_component = std::max(0, 128 - (max_depth + jumps) * jump_multiplier);
						voxel_color = vec4(255, other_component, other_component, 255);
					}
					else if(render_mode == NOT_NULL_JUMPS)
					{
						other_component = std::max(0, 128 - max_depth * jump_multiplier);
						voxel_color = vec4(other_component, other_component, 255, 255);
					}
					
					set_pixel(i, j, voxel_color);
					break;
				}
			}
			average_jumps += jumps;
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

	if(depth > max_depth)
		max_depth = depth;

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
			std::cout << "stack loop" << std::endl;
		}
	} while(octree_get_stack(v, rv, voxel_color, node_size, proportional_position, solid_voxel) < 2);

	return solid_voxel;
}

void octree_insert(node_t** node, vec3 point, vec4 color, vec3 position, int depth)
{
	if(*node == nullptr)
	{
		memory_usage += sizeof(node_t);
		solid_nodes[depth]++;
		*node = (node_t*) malloc(sizeof(node_t));
		for (int i = 0; i < 8; ++i)
			(*node)->children[i] = nullptr;
		(*node)->leaf = false;
		
		if(depth != octree->depth)
			octree_internal_nodes++;
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
					

					if(get_octree_voxel(ray_hit_offseted, ray_hit, voxel_color, node_size, proportional_position))
					{
						if(render_mode == DEPTH)
						{
							voxel_color = vec4(
								std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
								std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
								std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
								255);
						}
						else if(render_mode == COLOR)
						{
							voxel_color = voxel_color;
							voxel_color.w = 255;
						}
						else if(render_mode == SHADOW)
						{
							voxel_color = voxel_color;
							voxel_color *= float(dot(- face_normal, directional_light) * 0.25 + 0.75);
							voxel_color.w = 255;
						}
						else if(render_mode == JUMPS)
						{
							other_component = std::max(0, 128 - (max_depth + jumps) * jump_multiplier);
							voxel_color = vec4(255, other_component, other_component, 255);
						}
						else if(render_mode == NOT_NULL_JUMPS)
						{
							other_component = std::max(0, 128 - max_depth * jump_multiplier);
							voxel_color = vec4(other_component, other_component, 255, 255);
						}
						
						set_pixel(i, j, voxel_color);
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
				node_size = start_node_size;
				proportional_position = start_proportional_position;
			}

			// traversal
			vec3 step = vec3(1, 1, 1);						// Signo de cada componente del vector ray_direction
			vec3 deltaT = vec3(999999.0f, 999999.0f, 999999.0f);		// Distancia que cada componente avanzaría si el rayo recorriese 1 unidad en ese axis
			vec3 distance = vec3(999999.0f, 999999.0f, 999999.0f);		// Distancia que debe recorrer el rayo para chocar con la siguiente pared de ese axis

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
					voxel_incr.x = 1;
				else if((distance.y <= distance.x) && (distance.y <= distance.z))
					voxel_incr.y = 1;
				else if((distance.z <= distance.x) && (distance.z <= distance.y))
					voxel_incr.z = 1;

				total_dist += min_distance;
				ray_hit = camera_pos + ray_direction * total_dist;
				ray_hit_offseted = ray_hit + voxel_incr * step * 0.5f;
				jumps++;

				if(ray_hit_offseted.x < 0 || ray_hit_offseted.y < 0 || ray_hit_offseted.z < 0 ||
				ray_hit_offseted.x > bounding_box.x || ray_hit_offseted.y > bounding_box.y || ray_hit_offseted.z > bounding_box.z)
				{
					if(render_mode == JUMPS)
					{
						other_component = std::max(0, 255 - (max_depth + jumps) * jump_multiplier);
						voxel_color = vec4(255, other_component, other_component, 255);
					}
					else if(render_mode == NOT_NULL_JUMPS)
					{
						other_component = std::max(0, 255 - max_depth * jump_multiplier * 2);
						voxel_color = vec4(other_component, other_component, 255, 255);
					}
					else
						voxel_color = vec4(255, 255, 255, 255);

					set_pixel(i, j, voxel_color);
					break;
				}

				
				if(get_octree_voxel(ray_hit_offseted, ray_hit, voxel_color, node_size, proportional_position))
				{
					if(render_mode == DEPTH)
					{
						voxel_color = vec4(
							std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
							std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
							std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
							255);
					}
					else if(render_mode == COLOR)
					{
						voxel_color = voxel_color;
						face_normal = - voxel_incr;
						voxel_color.w = 255;
					}
					else if(render_mode == SHADOW)
					{
						voxel_color = voxel_color;
						voxel_color *= float(dot(voxel_incr, directional_light) * 0.25 + 0.75);
						voxel_color.w = 255;
					}
					else if(render_mode == JUMPS)
					{
						other_component = std::max(0, 128 - (max_depth + jumps) * jump_multiplier);
						voxel_color = vec4(255, other_component, other_component, 255);
					}
					else if(render_mode == NOT_NULL_JUMPS)
					{
						other_component = std::max(0, 128 - max_depth * jump_multiplier);
						voxel_color = vec4(other_component, other_component, 255, 255);
					}

					set_pixel(i, j, voxel_color);
					break;
				}
			}
			average_jumps += jumps;
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
					

					if(get_octree_voxel_stack(ray_hit_offseted, ray_hit, voxel_color, node_size, proportional_position))
					{
						if(render_mode == DEPTH)
						{
							voxel_color = vec4(
								std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
								std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
								std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
								255);
						}
						else if(render_mode == COLOR)
						{
							voxel_color = voxel_color;
							voxel_color.w = 255;
						}
						else if(render_mode == SHADOW)
						{
							voxel_color = voxel_color;
							voxel_color *= float(dot(- face_normal, directional_light) * 0.25 + 0.75);
							voxel_color.w = 255;
						}
						else if(render_mode == JUMPS)
						{
							other_component = std::max(0, 128 - (max_depth + jumps) * jump_multiplier);
							voxel_color = vec4(255, other_component, other_component, 255);
						}
						else if(render_mode == NOT_NULL_JUMPS)
						{
							other_component = std::max(0, 128 - max_depth * jump_multiplier);
							voxel_color = vec4(other_component, other_component, 255, 255);
						}

						set_pixel(i, j, voxel_color);
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
			vec3 deltaT = vec3(999999.0f, 999999.0f, 999999.0f);		// Distancia que cada componente avanzaría si el rayo recorriese 1 unidad en ese axis
			vec3 distance = vec3(999999.0f, 999999.0f, 999999.0f);		// Distancia que debe recorrer el rayo para chocar con la siguiente pared de ese axis

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
					if(render_mode == JUMPS)
					{
						other_component = std::max(0, 255 - (max_depth + jumps) * jump_multiplier);
						voxel_color = vec4(255, other_component, other_component, 255);
					}
					else if(render_mode == NOT_NULL_JUMPS)
					{
						other_component = std::max(0, 255 - max_depth * jump_multiplier * 2);
						voxel_color = vec4(other_component, other_component, 255, 255);
					}
					else
						voxel_color = vec4(255, 255, 255, 255);

					set_pixel(i, j, voxel_color);
					break;
				}

				if(get_octree_voxel_stack(ray_hit_offseted, ray_hit, voxel_color, node_size, proportional_position))
				{
					if(render_mode == DEPTH)
					{
						voxel_color = vec4(
							std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
							std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
							std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
							255);
					}
					else if(render_mode == COLOR)
					{
						voxel_color = voxel_color;
						face_normal = - voxel_incr;
						//voxel_color *= float(dot(-face_normal, directional_light) * 0.25 + 0.75);
						voxel_color.w = 255;
					}
					else if(render_mode == SHADOW)
					{
						voxel_color = voxel_color;
						voxel_color *= float(dot(voxel_incr, directional_light) * 0.25 + 0.75);
						voxel_color.w = 255;
					}
					else if(render_mode == JUMPS)
					{
						other_component = std::max(0, 128 - (max_depth + jumps) * jump_multiplier);
						voxel_color = vec4(255, other_component, other_component, 255);
					}
					else if(render_mode == NOT_NULL_JUMPS)
					{
						other_component = std::max(0, 128 - max_depth * jump_multiplier);
						voxel_color = vec4(other_component, other_component, 255, 255);
					}

					set_pixel(i, j, voxel_color);
					break;
				}
			}
			average_jumps += jumps;
		}
	}
}

void get_kdtree_init_variables(vec3 real_point, float &node_size_ptr, vec3 &proportional_position_ptr)
{
	kdtree->get_custom(
		clamp(real_point.x, 0.5f, BOUNDING_BOX_SIZE - 0.5f),
		clamp(real_point.y, 0.5f, BOUNDING_BOX_SIZE - 0.5f),
		clamp(real_point.z, 0.5f, BOUNDING_BOX_SIZE - 0.5f),
		real_point.x,
		real_point.y,
		real_point.z,
		node_size_ptr,
		proportional_position_ptr
		);
}

void get_kdtree_lidar_init_variables(vec3 real_point, float &node_size_ptr, vec3 &proportional_position_ptr)
{
	kdtree_lidar->get_custom(
		real_point.x,
		real_point.y,
		real_point.z,
		real_point.x,
		real_point.y,
		real_point.z,
		node_size_ptr,
		proportional_position_ptr
		);
}

bool get_kdtree_voxel(vec3 v, vec3 rv, vec4 &voxel_color, float &node_size, vec3 &proportional_position)
{
	std::vector<k3_tree_base<>::point_type> result;
	if(kdtree->get_custom(
		v.x,
		v.y,
		v.z,
		rv.x,
		rv.y,
		rv.z,
		node_size,
		proportional_position
		))
	{
		voxel_color = hsv2rgb(vec4(SimplexNoise::noise(int(v.x) * 0.01f, int(v.y) * 0.01f, int(v.z) * 0.01f) * 127.5f + 127.5f, 255, 255, 255));
		return true;
	}
	return false;
}

bool get_kdtree_lidar_voxel(vec3 v, vec3 rv, uint &voxel_color, float &node_size, vec3 &proportional_position)
{
	util_points::point_LIDAR_result result;
	if(kdtree_lidar->get_attr_no_factor_custom(
		v.x,
		v.y,
		v.z,
		rv.x,
		rv.y,
		rv.z,
		node_size,
		proportional_position,
		result
		))
	{
		voxel_color = result.intensity;
		return true;
	}
	return false;
}

void raycast_kdtree()
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
					ray_hit_offseted.x = clamp(ray_hit_offseted.x, 0.5f, BOUNDING_BOX_SIZE - 0.5f);
					ray_hit_offseted.y = clamp(ray_hit_offseted.y, 0.5f, BOUNDING_BOX_SIZE - 0.5f);
					ray_hit_offseted.z = clamp(ray_hit_offseted.z, 0.5f, BOUNDING_BOX_SIZE - 0.5f);
					total_dist = distance(camera_pos, ray_hit);
					

					if(get_kdtree_voxel(ray_hit_offseted, ray_hit, voxel_color, node_size, proportional_position))
					{
						max_depth = std::max(max_depth, tree_height - int(log(node_size) / log(2)));

						if(render_mode == DEPTH)
						{
							voxel_color = vec4(
								std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
								std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
								std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
								255);
						}
						else if(render_mode == COLOR)
						{
							voxel_color = voxel_color;
							voxel_color.w = 255;
						}
						else if(render_mode == SHADOW)
						{
							voxel_color = voxel_color;
							voxel_color *= float(dot(- face_normal, directional_light) * 0.25 + 0.75);
							voxel_color.w = 255;
						}
						else if(render_mode == JUMPS)
						{
							other_component = std::max(0, 128 - (max_depth + jumps) * jump_multiplier);
							voxel_color = vec4(255, other_component, other_component, 255);
						}
						else if(render_mode == NOT_NULL_JUMPS)
						{
							other_component = std::max(0, 128 - max_depth * jump_multiplier);
							voxel_color = vec4(other_component, other_component, 255, 255);
						}
						
						set_pixel(i, j, voxel_color);
						continue;
					}
					else
					{
						max_depth = std::max(max_depth, tree_height - int(log(node_size) / log(2)));
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
				node_size = start_node_size;
				proportional_position = start_proportional_position;
			}

			// traversal
			vec3 step = vec3(1, 1, 1);						// Signo de cada componente del vector ray_direction
			vec3 deltaT = vec3(999999.0f, 999999.0f, 999999.0f);		// Distancia que avanzaría el rayo para recorrer 1 unidad en ese axis
			vec3 distance = vec3(999999.0f, 999999.0f, 999999.0f);		// Distancia que debe recorrer el rayo para chocar con la siguiente pared de ese axis

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
				if((int) voxel_incr.x != 0 || proportional_position.x == 0 || proportional_position.x == 1)
					distance.x = node_size * deltaT.x;
				else if(step.x > 0)
					distance.x = (1.0f - proportional_position.x) * node_size * deltaT.x;
				else if(step.x < 0)
					distance.x = proportional_position.x * node_size * deltaT.x;
				else
					distance.x = 9999;

				if((int) voxel_incr.y != 0 || proportional_position.y == 0 || proportional_position.y == 1)
					distance.y = node_size * deltaT.y;
				else if(step.y > 0)
					distance.y = (1.0f - proportional_position.y) * node_size * deltaT.y;
				else if(step.y < 0)
					distance.y = proportional_position.y * node_size * deltaT.y;
				else
					distance.y = 9999;

				if((int) voxel_incr.z != 0 || proportional_position.z == 0 || proportional_position.z == 1)
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
				if(distance.x <= distance.y && distance.x <= distance.z)
					voxel_incr.x = step.x;
				else if(distance.y <= distance.x && distance.y <= distance.z)
					voxel_incr.y = step.y;
				else if(distance.z <= distance.x && distance.z <= distance.y)
					voxel_incr.z = step.z;

				total_dist += min_distance;
				ray_hit = camera_pos + ray_direction * total_dist;
				ray_hit_offseted = ray_hit + voxel_incr * 0.5f;
				jumps++;

				if(ray_hit_offseted.x < 0 || ray_hit_offseted.y < 0 || ray_hit_offseted.z < 0 ||
				ray_hit_offseted.x >= bounding_box.x || ray_hit_offseted.y >= bounding_box.y || ray_hit_offseted.z >= bounding_box.z)
				{
					//max_depth = std::max(max_depth, tree_height - int(log(node_size) / log(2)));

					if(render_mode == JUMPS)
					{
						other_component = std::max(0, 255 - (jumps + max_depth) * jump_multiplier);
						voxel_color = vec4(255, other_component, other_component, 255);
					}
					else if(render_mode == NOT_NULL_JUMPS)
					{
						other_component = std::max(0, 255 - max_depth * jump_multiplier * 2);
						voxel_color = vec4(other_component, other_component, 255, 255);
					}
					else
						voxel_color = vec4(255, 255, 255, 255);

					set_pixel(i, j, voxel_color);
					break;
				}

				
				if(get_kdtree_voxel(ray_hit_offseted, ray_hit, voxel_color, node_size, proportional_position))
				{
					max_depth = std::max(max_depth, tree_height - int(log(node_size) / log(2)));
					
					if(render_mode == DEPTH)
					{
						voxel_color = vec4(
							std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
							std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
							std::min(255, std::max(0, (int) (total_dist * 128.0f / BOUNDING_BOX_SIZE))),
							255);
					}
					else if(render_mode == COLOR)
					{
						voxel_color = voxel_color;
						voxel_color.w = 255;
					}
					else if(render_mode == SHADOW)
					{
						voxel_color = voxel_color;
						voxel_color *= float(dot(voxel_incr, directional_light) * 0.25 + 0.75);
						voxel_color.w = 255;
					}
					else if(render_mode == JUMPS)
					{
						other_component = std::max(0, 128 - (max_depth + jumps) * jump_multiplier);
						voxel_color = vec4(255, other_component, other_component, 255);
					}
					else if(render_mode == NOT_NULL_JUMPS)
					{
						other_component = std::max(0, 128 - max_depth * jump_multiplier);
						voxel_color = vec4(other_component, other_component, 255, 255);
					}

					set_pixel(i, j, voxel_color);
					break;
				}
				else
				{
					max_depth = std::max(max_depth, tree_height - int(log(node_size) / log(2)));
				}
			}
			average_jumps += jumps;
		}
	}
}

void raycast_kdtree_lidar()
{
	for (int i = 0; i < width_pixels; ++i)
	{
		for (int j = 0; j < height_pixels; ++j)
		{
			if((input_lclick_down && i == width_pixels / 2 && j == height_pixels / 2) ||
				(debugging && i == debugging_i && j == debugging_j))
			{
				debugging = false;
				int a = 0;
			}

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
					ray_hit_offseted.x = clamp(ray_hit_offseted.x, 0.5f, BOUNDING_BOX_SIZE - 0.5f);
					ray_hit_offseted.y = clamp(ray_hit_offseted.y, 0.5f, BOUNDING_BOX_SIZE - 0.5f);
					ray_hit_offseted.z = clamp(ray_hit_offseted.z, 0.5f, BOUNDING_BOX_SIZE - 0.5f);
					total_dist = distance(camera_pos, ray_hit);
					uint voxel_color;

					if(get_kdtree_lidar_voxel(ray_hit_offseted, ray_hit, voxel_color, node_size, proportional_position))
					{
						set_pixel(i, j, voxel_color);
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
				node_size = start_node_size;
				proportional_position = start_proportional_position;
			}

			// traversal
			vec3 step = vec3(1, 1, 1);						// Signo de cada componente del vector ray_direction
			vec3 deltaT = vec3(999999.0f, 999999.0f, 999999.0f);		// Distancia que avanzaría el rayo para recorrer 1 unidad en ese axis
			vec3 distance = vec3(999999.0f, 999999.0f, 999999.0f);		// Distancia que debe recorrer el rayo para chocar con la siguiente pared de ese axis

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
				if((int) voxel_incr.x != 0 || proportional_position.x == 0 || proportional_position.x == 1)
					distance.x = node_size * deltaT.x;
				else if(step.x > 0)
					distance.x = (1.0f - proportional_position.x) * node_size * deltaT.x;
				else if(step.x < 0)
					distance.x = proportional_position.x * node_size * deltaT.x;
				else
					distance.x = 9999;

				if((int) voxel_incr.y != 0 || proportional_position.y == 0 || proportional_position.y == 1)
					distance.y = node_size * deltaT.y;
				else if(step.y > 0)
					distance.y = (1.0f - proportional_position.y) * node_size * deltaT.y;
				else if(step.y < 0)
					distance.y = proportional_position.y * node_size * deltaT.y;
				else
					distance.y = 9999;

				if((int) voxel_incr.z != 0 || proportional_position.z == 0 || proportional_position.z == 1)
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
				if(distance.x <= distance.y && distance.x <= distance.z)
					voxel_incr.x = step.x;
				else if(distance.y <= distance.x && distance.y <= distance.z)
					voxel_incr.y = step.y;
				else if(distance.z <= distance.x && distance.z <= distance.y)
					voxel_incr.z = step.z;

				total_dist += min_distance;
				ray_hit = camera_pos + ray_direction * total_dist;
				ray_hit_offseted = ray_hit + voxel_incr * 0.5f;
				jumps++;

				if(ray_hit_offseted.x < 0 || ray_hit_offseted.y < 0 || ray_hit_offseted.z < 0 ||
				ray_hit_offseted.x >= bounding_box.x || ray_hit_offseted.y >= bounding_box.y || ray_hit_offseted.z >= bounding_box.z)
				{
					
					if(render_mode == JUMPS)
					{
						other_component = std::max(0, 255 - (max_depth + jumps) * jump_multiplier);
						voxel_color = vec4(255, other_component, other_component, 255);
					}
					else if(render_mode == NOT_NULL_JUMPS)
					{
						other_component = std::max(0, 255 - max_depth * jump_multiplier);
						voxel_color = vec4(other_component, other_component, 255, 255);
					}
					else
						voxel_color = vec4(255, 255, 255, 255);

					set_pixel(i, j, voxel_color);
					break;
				}

				uint voxel_color;
				if(get_kdtree_lidar_voxel(ray_hit_offseted, ray_hit, voxel_color, node_size, proportional_position))
				{
					set_pixel(i, j, voxel_color);
					break;
				}
			}
			average_jumps += jumps;
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
		if(data_type == OCTREE || data_type == OCTREE_STACK)
			//octree_set_default_stack(camera_pos, start_node_size, start_proportional_position);
			get_octree_node_size(&octree->root, camera_pos, camera_pos, vec3(), 0, &start_node_size, &start_proportional_position);
		else if(data_type == KDTREE)
			get_kdtree_init_variables(camera_pos, start_node_size, start_proportional_position);
		else if(data_type == KDTREE_LIDAR)
			get_kdtree_lidar_init_variables(camera_pos, start_node_size, start_proportional_position);
	}

	// write the pixels
	SDL_LockSurface(pixels);

	if(data_type == MATRIX)
		raycast_matrix();
	else if(data_type == OCTREE)
		raycast_octree();
	else if(data_type == OCTREE_STACK)
		raycast_octree_stack();
	else if(data_type == KDTREE)
		raycast_kdtree();
	else if(data_type == KDTREE_LIDAR)
		raycast_kdtree_lidar();

	rays += width_pixels * height_pixels;
	total_frames++;

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

void read_next_target()
{
	int previous_index = current_index;
	current_index = (current_index + 1) % target_positions.size();
	origin_camera_pos = target_camera_pos;

	while(target_phi > 180.0f)
		target_phi -= 360.0f;
	while(target_phi <= -180.0f)
		target_phi += 360.0f;

	origin_theta = target_theta;
	origin_phi = target_phi;

	origin_camera_pos = vec3(
		target_positions[previous_index][0] * BOUNDING_BOX_SIZE,
		target_positions[previous_index][1] * BOUNDING_BOX_SIZE,
		target_positions[previous_index][2] * BOUNDING_BOX_SIZE
	);
	target_camera_pos = vec3(
		target_positions[current_index][0] * BOUNDING_BOX_SIZE,
		target_positions[current_index][1] * BOUNDING_BOX_SIZE,
		target_positions[current_index][2] * BOUNDING_BOX_SIZE
	);
	target_theta = target_positions[current_index][3];
	target_phi = target_positions[current_index][4];

	target_phi = abs(target_phi - origin_phi) < abs(target_phi + 360.0f - origin_phi) ? target_phi : target_phi + 360.0f;
	target_phi = abs(target_phi - origin_phi) < abs(target_phi - 360.0f - origin_phi) ? target_phi : target_phi - 360.0f;

	float dist = distance(target_camera_pos, origin_camera_pos);
	lerp_speed = dist > 0 ? 1.0f / dist : 1.0f;
}

void read_target_positions()
{
	if(benchmark_name.find(".txt") == std::string::npos)
		benchmark_name = benchmark_name + ".txt";

	FILE * pFile;
	pFile = fopen(("benchmarks/" + benchmark_name).c_str(), "a");
	fclose(pFile);

	std::fstream infile;
	infile.open("benchmarks/" + benchmark_name, fstream::out | fstream::in);

	int total_lines = 0;
	target_positions.clear();

	std::string line;
	while (std::getline(infile, line))
	{
		total_lines++;
		float a, b, c, d, e;
		std::istringstream iss(line);
		if (!(iss >> a >> b >> c >> d >> e))
			break;
		vector<float> item;
		item.push_back(a);
		item.push_back(b);
		item.push_back(c);
		item.push_back(d);
		item.push_back(e);
		target_positions.push_back(item);
	}

	current_index = 0;
	if(total_lines > 0)
		read_next_target();
	
	infile.close();
}

void lerp_to_next_target()
{
	camera_pos = origin_camera_pos * (1.0f - lerp_value) + target_camera_pos * lerp_value;
	camera_angles.x = origin_theta * (1.0f - lerp_value) + target_theta * lerp_value;
	camera_angles.y = origin_phi * (1.0f - lerp_value) + target_phi * lerp_value;

	camera_forward = rotate(vec3(0, 0, 1), radians(camera_angles.x), vec3(-1, 0, 0));
	camera_forward = rotate(camera_forward, radians(camera_angles.y), vec3(0, -1, 0));

	camera_right = normalize(cross(camera_forward, world_up));
	camera_up = cross(camera_right, camera_forward);
}

void record_position()
{
	FILE *fptr;

	// use appropriate location if you are using MacOS or Linux
	fptr = fopen(("benchmarks/" + benchmark_name).c_str(),"a");

	if(fptr == NULL)
	{
		printf("Error!");   
		exit(1);             
	}

	fprintf(fptr,"%.1f %.1f %.1f %.1f %.1f\n",
		camera_pos.x / BOUNDING_BOX_SIZE,
		camera_pos.y / BOUNDING_BOX_SIZE,
		camera_pos.z / BOUNDING_BOX_SIZE,
		camera_angles.x,
		camera_angles.y
		);
	fclose(fptr);

	//read_target_positions();
	vector<float> item;
	item.push_back(camera_pos.x / BOUNDING_BOX_SIZE);
	item.push_back(camera_pos.y / BOUNDING_BOX_SIZE);
	item.push_back(camera_pos.z / BOUNDING_BOX_SIZE);
	item.push_back(camera_angles.x);
	item.push_back(camera_angles.y);
	target_positions.push_back(item);
}

string print_dimensionality()
{
	float dimensionality;
	if(data_type != KDTREE)
	{
		float average_proportion = 0;
		for(int i = 1; i <= tree_height; i++)
			average_proportion += solid_nodes[i] / (float) solid_nodes[i - 1];
		average_proportion /= tree_height;

		dimensionality = log2f(average_proportion);
	}
	else
		dimensionality = kdtree->get_dimensionality(solid_nodes, tree_height);

	//cout << "average dimensionality: " << dimensionality << endl;
	std::stringstream stream;
	stream << std::fixed << std::setprecision(2) << dimensionality;
	return stream.str();
}

string print_internal_nodes()
{
	if(data_type == OCTREE)
	{
		//std::cout << "internal nodes: " << octree_internal_nodes << std::endl;
		return to_string(octree_internal_nodes);
	}
	else if(data_type == KDTREE)
	{
		//std::cout << "internal nodes: " << kdtree->get_k_t_length() / 8 << std::endl;
		return to_string(kdtree->get_k_t_length() / 8);
	}
	return "";
}

string print_space_divisions()
{
	if(data_type == OCTREE)
	{
		//std::cout << "space divisions: " << octree_internal_nodes << std::endl;
		return to_string(octree_internal_nodes);
	}
	else if(data_type == KDTREE)
	{
		//std::cout << "space divisions: " << (kdtree->get_k_t_length() + kdtree->get_k_l_length()) / 8 << std::endl;
		return to_string((kdtree->get_k_t_length() + kdtree->get_k_l_length()) / 8);
	}
	return "";
}

string print_memory_usage()
{
	float memory_usage_trad;

	if(data_type != KDTREE)
		memory_usage_trad = memory_usage;
	else
	{
		std::fstream infile;
		infile.open("trash_output.txt", fstream::out | fstream::in);
		memory_usage_trad = (unsigned long long) kdtree->serialize(infile, NULL, "") * 32;
	}

	//int divisions = 0;
	//while(memory_usage_trad / 1024 >= 1)
	//{
	//	memory_usage_trad = memory_usage_trad / 1024.0f;
	//	divisions++;
	//}

	//string unit[] = {"bytes", "kilobytes", "megabytes", "gigabytes", "terabytes"};

	//printf("using %.2f %s.\n", memory_usage_trad, unit[divisions].c_str());

	std::stringstream stream;
	stream << std::fixed << std::setprecision(2) << memory_usage_trad;
	return stream.str();
}

void load_mesh()
{
	if(mesh_filename.find(".stl") == std::string::npos)
		mesh_filename = mesh_filename + ".stl";
	StlMesh <float, unsigned int> mesh ("models/" + mesh_filename);

	// MESH MAPPING PART
	// prefix g is for geometry, the values for the bounding box that contains the mesh entirely
	// aabb: axis aligned bounding box

	// this values will help to find the whole geometry's bounding box
	vec3 g_aabb_min = vec3( 9999.0f,  9999.0f,  9999.0f);
	vec3 g_aabb_max = vec3(-9999.0f, -9999.0f, -9999.0f);

	vec3 vertices[3];
	vec3 g_angles = vec3(rx, ry, rz);

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

	// variables in case KDTREE is the current data_type
	// Read file and create conceptual tree
	uint64_t pos_x, pos_y, pos_z;
	uint64_t size_x = 0, size_y = 0, size_z = 0;

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
					else if(data_type == OCTREE || data_type == OCTREE_STACK)
						octree_insert(
							&octree->root,
							vec3(i, j, k),
							hsv2rgb(vec4(hue, 255, 255, 255)),
							vec3(),
							0
						);
					else if(data_type == KDTREE)
					{
						//voxel_data[i * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE + j * BOUNDING_BOX_SIZE + k] = hsv2rgb(vec4(hue, 255, 255, 255));
						
						// Get position (x, y, z)
						pos_x = i;
						pos_y = j;
						pos_z = k;

						// Store point into vector
						points.push_back({pos_x, pos_y, pos_z,});

						// Calculate max size
						size_x = std::max(size_x, pos_x);
						size_y = std::max(size_y, pos_y);
						size_z = std::max(size_z, pos_z);
					}
					else if(data_type == KDTREE_LIDAR)
					{
						//voxel_data[i * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE + j * BOUNDING_BOX_SIZE + k] = hsv2rgb(vec4(hue, 255, 255, 255));
						
						// Get position (x, y, z)
						pos_x = i;
						pos_y = j;
						pos_z = k;

						// Store point into vector
						util_points::point_LIDAR point_l{};
						point_l.X = i;
						point_l.Y = j;
						point_l.Z = k;
						point_l.intensity = hex(hsv2rgb(vec4(hue, 255, 255, 255)));
						points_lidar.push_back(point_l);

						// Calculate max size
						size_x = std::max(size_x, pos_x);
						size_y = std::max(size_y, pos_y);
						size_z = std::max(size_z, pos_z);
					}
				}
	}

	if(data_type == KDTREE)
	{
		// end of case KDTREE is the current data_type
		// Remove duplicates
		std::sort(points.begin(), points.end());
		points.erase(std::unique(points.begin(), points.end()), points.end());

		kdtree = new k3_tree(points, size_x + 1, size_y + 1, size_z + 1, 2, 2, 2);
	}
	if(data_type == KDTREE_LIDAR)
	{
		std::sort(points_lidar.begin(), points_lidar.end());
		points_lidar.erase(std::unique(points_lidar.begin(), points_lidar.end()), points_lidar.end());

		kdtree_lidar = new k3_tree_Lp1(points_lidar);
	}
}

void initialize()
{
	bounding_box = vec3(BOUNDING_BOX_SIZE, BOUNDING_BOX_SIZE, BOUNDING_BOX_SIZE);
	solid_nodes = (int*) malloc((tree_height + 1) * sizeof(int));
	for(int i = 0; i < tree_height + 1; i++)
		solid_nodes[i] = 0;

	if(data_type == MATRIX)
	{
		//voxel_data = new vec4[BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE];
		memory_usage += BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE * sizeof(vec4);
		voxel_data = (vec4*) malloc(BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE * sizeof(vec4));
		for(uint i = 0; i < BOUNDING_BOX_SIZE; i++)
			for(uint j = 0; j < BOUNDING_BOX_SIZE; j++)
				for(uint k = 0; k < BOUNDING_BOX_SIZE; k++)
					voxel_data[i * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE + j * BOUNDING_BOX_SIZE + k] = vec4();
	}
	else if(data_type == OCTREE || data_type == OCTREE_STACK)
	{
		// Octree
		memory_usage += sizeof(octree_t);
		octree = (octree_t*) malloc(sizeof(octree_t));
		octree->size = BOUNDING_BOX_SIZE;
		octree->depth = round(log2(double(BOUNDING_BOX_SIZE)));
		memory_usage += sizeof(node_t);
		octree_internal_nodes++;
		solid_nodes[0]++;
		octree->root = (node_t*) malloc(sizeof(node_t));
		for (int i = 0; i < 8; ++i)
			octree->root->children[i] = nullptr;
		octree->root->leaf = false;

		stack_node_t root_item;
		root_item.node = octree->root;
		root_item.position = vec3();

		if(data_type == OCTREE_STACK)
			default_stack.push(root_item);
	}

	if(mesh_type == NOISE || mesh_type == NOISE_SOLID)
	{
		// Read file and create conceptual tree
		uint64_t pos_x, pos_y, pos_z;
		uint64_t size_x = 0, size_y = 0, size_z = 0;

		for(uint i = 0; i < BOUNDING_BOX_SIZE; i++)
			for(uint j = 0; j < BOUNDING_BOX_SIZE; j++)
				for(uint k = 0; k < BOUNDING_BOX_SIZE; k++)
				{
					float noise_value = SimplexNoise::noise(i * 0.02f, j * 0.02f, k * 0.02f);
					int hidden = 0;

					bool solid = noise_value < noise_value_higher && noise_value > noise_value_lower;
					if(mesh_type != NOISE_SOLID && solid)
					{
						if(i > 0)
						{
							noise_value = SimplexNoise::noise((i - 1) * 0.02f, j * 0.02f, k * 0.02f);
							if(noise_value < noise_value_higher && noise_value > noise_value_lower)
								hidden++;
						}
						if(i < BOUNDING_BOX_SIZE - 1)
						{
							noise_value = SimplexNoise::noise((i + 1) * 0.02f, j * 0.02f, k * 0.02f);
							if(noise_value < noise_value_higher && noise_value > noise_value_lower)
								hidden++;
						}
						if(j > 0)
						{
							noise_value = SimplexNoise::noise(i * 0.02f, (j - 1) * 0.02f, k * 0.02f);
							if(noise_value < noise_value_higher && noise_value > noise_value_lower)
								hidden++;
						}
						if(j < BOUNDING_BOX_SIZE - 1)
						{
							noise_value = SimplexNoise::noise(i * 0.02f, (j + 1) * 0.02f, k * 0.02f);
							if(noise_value < noise_value_higher && noise_value > noise_value_lower)
								hidden++;
						}
						if(k > 0)
						{
							noise_value = SimplexNoise::noise(i * 0.02f, j * 0.02f, (k - 1) * 0.02f);
							if(noise_value < noise_value_higher && noise_value > noise_value_lower)
								hidden++;
						}
						if(k < BOUNDING_BOX_SIZE - 1)
						{
							noise_value = SimplexNoise::noise(i * 0.02f, j * 0.02f, (k + 1) * 0.02f);
							if(noise_value < noise_value_higher && noise_value > noise_value_lower)
								hidden++;
						}
					}

					if(solid && hidden < 6)
					{
						float hue_noise_value = SimplexNoise::noise(i * 0.01f, j * 0.01f, k * 0.01f);
						int hue = int(hue_noise_value * 127.5f + 127.5f);
						if(data_type == MATRIX)
							voxel_data[i * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE + j * BOUNDING_BOX_SIZE + k] = hsv2rgb(vec4(hue, 255, 255, 255));
						else if(data_type == OCTREE || data_type == OCTREE_STACK) 
							octree_insert(
								&octree->root,
								vec3(i, j, k),
								hsv2rgb(vec4(hue, 255, 255, 255)),
								vec3(),
								0
							);
						else if(data_type == KDTREE)
						{
							// Get position (x, y, z)
							pos_x = i;
							pos_y = j;
							pos_z = k;

							// Store point into vector
							points.push_back({pos_x, pos_y, pos_z});

							// Calculate max size
							size_x = std::max(size_x, pos_x);
							size_y = std::max(size_y, pos_y);
							size_z = std::max(size_z, pos_z);
						}
						else if(data_type == KDTREE_LIDAR)
						{
							// Get position (x, y, z)
							pos_x = i;
							pos_y = j;
							pos_z = k;

							// Store point into vector
							util_points::point_LIDAR point_l{};
							point_l.X = i;
							point_l.Y = j;
							point_l.Z = k;
							point_l.intensity = hex(hsv2rgb(vec4(hue, 255, 255, 255)));
							points_lidar.push_back(point_l);

							// Calculate max size
							size_x = std::max(size_x, pos_x);
							size_y = std::max(size_y, pos_y);
							size_z = std::max(size_z, pos_z);
						}
					}
					else
					{
						if(data_type == MATRIX)
							voxel_data[i * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE + j * BOUNDING_BOX_SIZE + k] = vec4(0, 0, 0, 0);
					}
				}

		if(data_type == KDTREE)
		{
			// Remove duplicates
			std::sort(points.begin(), points.end());
			points.erase(std::unique(points.begin(), points.end()), points.end());

			kdtree = new k3_tree(points, size_x + 1, size_y + 1, size_z + 1, 2, 2, 2);
		}

		if(data_type == KDTREE_LIDAR)
		{
			// Remove duplicates
			std::sort(points_lidar.begin(), points_lidar.end());
			points.erase(std::unique(points.begin(), points.end()), points.end());

			kdtree_lidar = new k3_tree_Lp1(points_lidar);
		}
	}
	else if(mesh_type == MESH)
	{
		load_mesh();
	}
	else if(mesh_type == RANDOM)
	{
		if(data_type == MATRIX)
		{
			for(int q = 0; q < random_points_count; q++)
			{
				int i = rand() % BOUNDING_BOX_SIZE;
				int j = rand() % BOUNDING_BOX_SIZE;
				int k = rand() % BOUNDING_BOX_SIZE;
				voxel_data[i * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE + j * BOUNDING_BOX_SIZE + k] = hsv2rgb(vec4(rand() % 255, 255, 255, 255));
			}
		}
		else if(data_type == OCTREE || data_type == OCTREE_STACK)
		{
			for(int i = 0; i < random_points_count; i++)
			{
				octree_insert(
					&octree->root,
					vec3(rand() % BOUNDING_BOX_SIZE, rand() % BOUNDING_BOX_SIZE, rand() % BOUNDING_BOX_SIZE),
					hsv2rgb(vec4(rand() % 255, 255, 255, 255)),
					vec3(),
					0
				);
			}
		}
		else if(data_type == KDTREE)
		{
			// Read file and create conceptual tree
			uint64_t pos_x, pos_y, pos_z;
			uint64_t size_x = 0, size_y = 0, size_z = 0;

			for(int i = 0; i < random_points_count; i++)
			{
				// Get position (x, y, z)
				pos_x = rand() % BOUNDING_BOX_SIZE;
				pos_y = rand() % BOUNDING_BOX_SIZE;
				pos_z = rand() % BOUNDING_BOX_SIZE;

				// Store point into vector
				points.push_back({pos_x, pos_y, pos_z});

				// Calculate max size
				size_x = std::max(size_x, pos_x);
				size_y = std::max(size_y, pos_y);
				size_z = std::max(size_z, pos_z);
			}

			// Remove duplicates
			std::sort(points.begin(), points.end());
			points.erase(std::unique(points.begin(), points.end()), points.end());

			kdtree = new k3_tree(points, size_x + 1, size_y + 1, size_z + 1, 2, 2, 2);
		}
		else if(data_type == KDTREE_LIDAR)
		{
			// Read file and create conceptual tree
			uint64_t pos_x, pos_y, pos_z;

			for(int i = 0; i < random_points_count; i++)
			{
				// Get position (x, y, z)
				pos_x = rand() % BOUNDING_BOX_SIZE;
				pos_y = rand() % BOUNDING_BOX_SIZE;
				pos_z = rand() % BOUNDING_BOX_SIZE;

				// Store point into vector
				util_points::point_LIDAR point_l{};
				point_l.X = pos_x;
				point_l.Y = pos_y;
				point_l.Z = pos_z;
				point_l.intensity = hex(hsv2rgb(vec4(rand() % 255, 255, 255, 255)));
				points_lidar.push_back(point_l);
			}

			// Remove duplicates
			std::sort(points_lidar.begin(), points_lidar.end());
			points_lidar.erase(std::unique(points_lidar.begin(), points_lidar.end()), points_lidar.end());

			kdtree_lidar = new k3_tree_Lp1(points_lidar);
		}
	}
	
	if(!input_recreate)
	{
		camera_pos = vec3(1.5, 1.5, -1.5);
		camera_forward = normalize(vec3(0, 0, 1));
		camera_right = cross(camera_forward, world_up);
		camera_up = cross(camera_right, camera_forward);
		camera_angles = vec2(0, 0);
		redraw();
	}

	for(int i = 0; i < 120; i++)
		fps[i] = 0;
}

void update()
{
	if((playing || benchmark_mode) && target_positions.size() > 0)
	{
		lerp_value += delta_time * lerp_speed * lerp_speed_multiplier;

		while(lerp_value >= 1.0f)
		{
			read_next_target();
			lerp_value -= 1.0f;
		}
		
		lerp_to_next_target();
	}
	else
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

		vec3 add = input_forward ? camera_forward : vec3();
		add -= input_left ? camera_right : vec3();
		add -= input_back ? camera_forward : vec3();
		add += input_right ? camera_right : vec3();
		add += input_up ? camera_up : vec3();
		add -= input_down ? camera_up : vec3();

		camera_pos += add * delta_time * (input_shift ? BOUNDING_BOX_SIZE : BOUNDING_BOX_SIZE * 0.25f);
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
	}
	
	if(render_mode == SHADOW)
		directional_light = rotateY(directional_light, delta_time);
		
	if(input_recreate)
	{
		initialize();
		input_recreate = false;
	}

	if(input_rclick_down)
		render_mode = (render_mode_enum) (((int) render_mode + 1) % 5);

	if(hide_mouse)
		SDL_WarpMouseInWindow(window, WIDTH / 2, HEIGHT / 2);
}

void catch_input()
{
	input_lclick_down = false;
	input_mclick_down = false;
	input_rclick_down = false;
	input_memory_usage_print_down = false;

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
			else if(event.key.keysym.scancode == SDL_SCANCODE_Z) playing = true;
			else if(event.key.keysym.scancode == SDL_SCANCODE_M)
			{
				input_memory_usage_print_down = input_memory_usage_print_down_reset;
				input_memory_usage_print = true;
				input_memory_usage_print_down_reset = false;
			}
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
			else if(event.key.keysym.scancode == SDL_SCANCODE_Z) playing = false;
			else if(event.key.keysym.scancode == SDL_SCANCODE_M)
			{
				input_memory_usage_print = false;
				input_memory_usage_print_down_reset = true;
			}
			else if(event.key.keysym.scancode == SDL_SCANCODE_LSHIFT) input_shift = false;
		}
		else if(event.type == SDL_MOUSEMOTION)
		{
			camera_angles.x = std::min(89.9f, std::max(-89.9f, camera_angles.x - event.motion.yrel * mouse_sens));
			camera_angles.y += event.motion.xrel * mouse_sens;
			while(camera_angles.y < -180.0f)
				camera_angles.y += 360.0f;
			while(camera_angles.y >= 180.0f)
				camera_angles.y -= 360.0f;

			camera_forward = rotate(vec3(0, 0, 1), radians(camera_angles.x), vec3(-1, 0, 0));
			camera_forward = rotate(camera_forward, radians(camera_angles.y), vec3(0, -1, 0));

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
			if(event.button.button == SDL_BUTTON_MIDDLE)
			{
				input_mclick_down = input_mclick_down_reset;
				input_mclick = true;
				input_mclick_down_reset = false;
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
			if(event.button.button == SDL_BUTTON_MIDDLE)
			{
				input_mclick_down_reset = true;
				input_mclick = false;
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
				jump_multiplier = std::min(40, jump_multiplier + 2);
			else if(event.wheel.y < 0)
				jump_multiplier = std::max(0, jump_multiplier - 2);
		}
	}
}

void run()
{
	uint it = 0;
	while(!input_quit)
	{
		if(!debugging)
		{
			catch_input();
			if(input_mclick_down)
				record_position();
			if(input_memory_usage_print_down)
				print_memory_usage();
			update();
		}

		if(draw_again)
		{
			draw();
			if(!debugging)
				draw_again = false;
		}

		if(!first_frame)
		{
			chrono::steady_clock::time_point current_time = chrono::steady_clock::now();
			delta_time = chrono::duration_cast<chrono::duration<float>>(current_time - previous_time).count();
			previous_time = current_time;
			smooth_delta_time = 0.8f * smooth_delta_time + 0.2 * delta_time;

			time_to_next_sample -= delta_time;
			if(time_to_next_sample <= 0.0f)
			{
				int added_samples = int(- time_to_next_sample / time_between_samples) + 1;
				current_samples += added_samples;
				if(benchmark_mode && current_samples >= target_samples)
					input_quit = true;
				fps[(int) clamp(round(1.0f / delta_time), 0, 119)] += added_samples;
				time_to_next_sample = time_between_samples;
			}
		}
		else
		{
			previous_time = chrono::steady_clock::now();
			first_frame = false;
		}

		if(it++ % int(std::max(1.0f, 0.2f / delta_time)) == 0)
			SDL_SetWindowTitle(window, to_string((int) (1.0 / smooth_delta_time)).c_str());
	}
}

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
			
			i++;
		}
	}

	width_pixels = WIDTH * RENDER_SCALE;
	height_pixels = HEIGHT * RENDER_SCALE;
	tree_height = log(BOUNDING_BOX_SIZE) / log(2);
	lerp_speed_multiplier = BOUNDING_BOX_SIZE;

	//srand(time(NULL));

	window = SDL_CreateWindow("Voxel", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_RESIZABLE);
	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

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

		cout << "internal nodes: " << print_internal_nodes() << endl;
		cout << "memory usage: " << print_memory_usage() << " bytes" << endl;
		cout << "dimensionality: " << print_dimensionality() << endl;
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
		//std::cout << "total samples: " << total_samples << std::endl;
		//std::cout << "average fps: " << average_fps << std::endl;

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
		//std::cout << "average 10% low fps: " << average_p10_fps << std::endl;

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
		//std::cout << "average 1% low fps: " << average_p1_fps << std::endl;

		float standart_deviation = 0;
		for(int i = 0; i < 120; i++)
			standart_deviation += fps[i] * pow(i - average_fps, 2);
		standart_deviation /= total_samples;
		standart_deviation = sqrt(standart_deviation);
		//std::cout << "standart_deviation: " << standart_deviation << std::endl;

		//std::cout << "Mrays per second: " << rays / duration / 1000000.0f << std::endl;

		//std::cout << "average jumps: " << average_jumps / (float) total_frames / (float) width_pixels / (float) height_pixels << std::endl;

		//cout << "internal nodes: " << print_internal_nodes() << endl;
		//cout << "memory usage: " << print_memory_usage() << " bytes" << endl;
		//cout << "dimensionality: " << print_dimensionality() << endl;

		
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

		//fprintf(fptr,"dataset,data_type,size,resolution_scale,average_fps,low10_fps,low1_fps,std_dev,Mrps,average_jumps,internal_nodes,memory_usage,dimensionality\n");
	
		string extension = ".stl";
		string name = mesh_filename;
		std::size_t ind = name.find(extension); // Find the starting position of substring in the string
		if(ind !=std::string::npos)
			name.erase(ind,extension.length());

		fprintf(fptr,"%s,%s,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%s,%s\n",
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
			print_internal_nodes().c_str(),
			print_memory_usage().c_str(),
			print_dimensionality().c_str()
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