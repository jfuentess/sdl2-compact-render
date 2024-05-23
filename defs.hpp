#define NOMINMAX

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/projection.hpp>
#include <k3_tree.hpp>
#include <k3_tree_base.hpp>

using namespace std;
using namespace glm;
using namespace k3tree;

#define WIDTH 1280
#define HEIGHT 720


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

std::vector<k3_tree_base<>::point_type> points;

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
