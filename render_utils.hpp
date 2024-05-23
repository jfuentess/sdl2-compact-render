#include "defs.hpp"

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

unsigned int hex(vec4 color)
{
  return ((int(color.w) & 0xff) << 24) + ((int(color.z) & 0xff) << 16) + ((int(color.y) & 0xff) << 8) + (int(color.x) & 0xff);
}

void set_pixel(int i, int j, vec4 color)
{
  unsigned int* row = (unsigned int*) ((char*) pixels->pixels + pitch * (height_pixels - 1 - j));
  row[i] = hex(color);
}

void set_pixel(int i, int j, uint color)
{
  unsigned int* row = (unsigned int*) ((char*) pixels->pixels + pitch * (height_pixels - 1 - j));
  row[i] = color;
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
