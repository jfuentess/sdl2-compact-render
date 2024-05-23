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
