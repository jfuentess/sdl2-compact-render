#include "render_utils.hpp"

vec4 get_matrix_voxel(vec3 v)
{
  if((int) v.x >= 0 && (int) v.x < BOUNDING_BOX_SIZE && (int) v.y >= 0 && (int) v.y < BOUNDING_BOX_SIZE && (int) v.z >= 0 && (int) v.z < BOUNDING_BOX_SIZE)
    return voxel_data[(int) v.x * BOUNDING_BOX_SIZE * BOUNDING_BOX_SIZE + (int) v.y * BOUNDING_BOX_SIZE + (int) v.z];
  return vec4(0, 0, 0, 0);
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
