#include "matrix_render.hpp"
#include "octree_render.hpp"
#include "k3tree_render.hpp"
#include "stl_reader.h"

using namespace stl_reader;

void redraw()
{
  draw_again = true;
}

void draw()
{
  outside = camera_pos.x < 0 || camera_pos.x > bounding_box.x ||
    camera_pos.y < 0 || camera_pos.y > bounding_box.y ||
    camera_pos.z < 0 || camera_pos.z > bounding_box.z;
		
  if(!outside)
    {
      if(data_type == OCTREE || data_type == OCTREE_STACK)
	get_octree_node_size(&octree->root, camera_pos, camera_pos, vec3(), 0, &start_node_size, &start_proportional_position);
      else if(data_type == KDTREE)
	get_kdtree_init_variables(camera_pos, start_node_size, start_proportional_position);
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
	    }
    }

  if(data_type == KDTREE)
    {
      // end of case KDTREE is the current data_type
      // Remove duplicates
      std::sort(points.begin(), points.end());
      points.erase(std::unique(points.begin(), points.end()), points.end());

      kdtree = new k3_tree<>(points, size_x + 1, size_y + 1, size_z + 1, 2, 2, 2);
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

	  kdtree = new k3_tree<>(points, size_x + 1, size_y + 1, size_z + 1, 2, 2, 2);
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

	  kdtree = new k3_tree<>(points, size_x + 1, size_y + 1, size_z + 1, 2, 2, 2);
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
