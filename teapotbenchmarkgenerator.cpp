// teapot benchmark generator

FILE *fptr;
fptr = fopen(("/home/zso/Dev/VOXEL/" + benchmark_name).c_str(),"w");

if(fptr == NULL)
{
	printf("Error!");   
	exit(1);             
}

for(float i = 0; i < 360; i += 2.5f)
{
	camera_pos.x = 0.5f + 0.7f * cosf(i * 3.141592f / 180.0f);
	camera_pos.y = 0.85f;
	camera_pos.z = 0.5f + 0.7f * sinf(i * 3.141592f / 180.0f);
	camera_angles.x = -30;
	camera_angles.y = i + 90;
	
	std::cout << camera_pos.x << " " << camera_pos.y << " " << camera_pos.z << std::endl;
	std::cout << camera_angles.x << " " << camera_angles.y << std::endl;

	fprintf(fptr,"%f %f %f %f %f\n",
		camera_pos.x,
		camera_pos.y,
		camera_pos.z,
		camera_angles.x,
		camera_angles.y
		);
}

fclose(fptr);