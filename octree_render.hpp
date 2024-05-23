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
