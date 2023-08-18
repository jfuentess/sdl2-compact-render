# K3-TREE-VOXEL

## Requisitos:
SDL2      (https://www.libsdl.org/)  
SDSL      (https://github.com/simongog/sdsl-lite)  
k3-LIDAR  (https://gitlab.lbd.org.es/fsilva/k3-lidar)  


## Guía de ejecución:

ejecución sugerida: ./main -rx -90

ejecutar con: ./main  
flags:  
	-s [n]  resolución del modelo, 64 default  
	-r [n]  escala de resolución pantalla, 0.2 default  
	-d [0, 1, 2, 3]  estructura de datos usada, 3 default  
		0:array3D  
		1:octree  
		2:octree_alt  
		3:k3-tree  
	-m [0, 1, 2, 3]  tipo de modelo a usar, 2 default  
		0:noise  
		1:noise_solid  
		2:mesh  
		3:random  
	-mf [filename.stl]  nombre del modelo en formato STL a abrir (sólo si se usó flag -m 2)  
	-p [n]  cantidad de puntos random a insertar (sólo si se usó flag -m 3)  
	-rx [n] rotación del modelo en el eje x (en grados)  
	-ry [n] rotación del modelo en el eje y (en grados)  
	-rz [n] rotación del modelo en el eje z (en grados)  

## Compilación:

ajustar paths con los correspondientes de su instalación

g++ -O3 -g -o main main.cpp SimplexNoise.cpp  
-I[SDL2]  
-I[k3-lidar/include]  
-I/[k3-lidar/external]   
-I[k3-lidar/external/sdsl-lite/include]  
-I[k3-lidar/external/LAStools/LASzip/src]  
-I[k3-lidar/external/malloc_count/include]  
-L[lib] -L[k3-lidar/external/LAStools/LASlib/lib]  
-lsdsl -lLASlib  
