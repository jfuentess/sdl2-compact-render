# K3-TREE-VOXEL

## Requisitos:
  SDL2      (https://www.libsdl.org/)__
  SDSL      (https://github.com/simongog/sdsl-lite)__
  k3-LIDAR  (https://gitlab.lbd.org.es/fsilva/k3-lidar)__


## Guía de ejecución:

  ejecución sugerida: ./main -rx -90
  
  ejecutar con: ./main__
  flags:__
    -s [n]  resolución del modelo, 64 default__
    -r [n]  escala de resolución pantalla, 0.2 default__
    -d [0, 1, 2, 3]  estructura de datos usada, 3 default__
        0:array3D__
        1:octree__
        2:octree_alt__
        3:k3-tree__
    -m [0, 1, 2, 3]  tipo de modelo a usar, 2 default__
        0:noise__
        1:noise_solid__
        2:mesh__
        3:random__
    -mf [filename.stl]  nombre del modelo en formato STL a abrir (sólo si se usó flag -m 2)__
    -p [n]  cantidad de puntos random a insertar (sólo si se usó flag -m 3)__
    -rx [n] rotación del modelo en el eje x (en grados)__
    -ry [n] rotación del modelo en el eje y (en grados)__
    -rz [n] rotación del modelo en el eje z (en grados)__

## Compilación:

  ajustar paths con los correspondientes de su instalación

  g++ -O3 -g -o main main.cpp SimplexNoise.cpp__
  `sdl2-config --cflags --libs`__
  -I[k3-lidar/include]__
  -I/[k3-lidar/external] __
  -I[k3-lidar/external/sdsl-lite/include]__
  -I[k3-lidar/external/LAStools/LASzip/src]__
  -I[k3-lidar/external/malloc_count/include]__
  -L[lib] -L[k3-lidar/external/LAStools/LASlib/lib]__
  -lsdsl -lLASlib__
