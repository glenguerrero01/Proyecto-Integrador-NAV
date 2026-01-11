# Proyecto-Integrador-NAV - Grupo 4
Repositorio proyecto integrador por William Andrade, Glen Guerrero y Yeison Jimenez

## *Parte 1* - Algortimo de localización con filtro EKF
Se ha implementado a la función analyze_bag un filtro EKF aplicado a la odometría obtenida a través del Rosbag; AMCL ya estaba integrado en el código.

<img width="741" height="516" alt="image" src="https://github.com/user-attachments/assets/a80413e9-d241-4ef8-8224-d437ac9c6d69" />
<img width="762" height="516" alt="image" src="https://github.com/user-attachments/assets/841d2fcb-6286-4f42-900a-904b4ec88a89" />

## *Parte 2* - Algoritmo de LidarSlam y mapa de ocupación
Se ha implementado un algoritmo de LidarSlam en el cual se consideran diferentes variables
- Map resolution
- Max Lidar range
- Loop closure threshold
- Loop closure searchradius
- Rango inicial
- Rango final
Como rango inicial se tomo aproximadamente desde el escaneo 15 que es el momento en el que el robot comienza a mover según la odometría, y se lo llevó hasta el escaneo #35 para realizar ajustes iniciales, se puede llevar a un mayor escaneo para posiblemente mejorar el mapa final. En resolución de mapa se realizaron diferentes pruebas, con resoluciones desde 1 hasta 30, siendo 15 el más optimo teniendo en cuenta el tiempo de procesamient (un mayor número representa mayor tiempo), siendo así que las otras variable como loop closure son importantes para ajustar el algoritmo, se tomo valores de 1 a 10, encontrando que los mejores estarían entre 2 y 3. con un umbral de 210. A continuación se indica las diferentes salidas de las pruebas a través de la graficación de la lectura del sensor lidar(scan en la primera gráfica en morado) y la construcción del mapa de ocupación (Occupancy grid) que puede ser necesario para la planificación de trayectorias.

### *Prueba 1*
<img width="850" height="485" alt="untitled1" src="https://github.com/user-attachments/assets/2ad2ea9f-8508-4c74-9ad5-66e7a306e564" />
<img width="752" height="515" alt="untitled2_1" src="https://github.com/user-attachments/assets/d01a5251-622d-4ca4-80b6-af07b347e97f" />

### *Prueba 2*
<img width="857" height="485" alt="untitled3" src="https://github.com/user-attachments/assets/4b8ce86d-dc2b-4408-9409-528488659670" />
<img width="693" height="478" alt="image" src="https://github.com/user-attachments/assets/3fa9951e-c251-45f2-a4cf-a193ddf05cf4" />

### *Prueba 3*
<img width="857" height="485" alt="untitled7" src="https://github.com/user-attachments/assets/c3c50f5a-953f-46da-8ee6-e2e43e22eb76" />
<img width="857" height="485" alt="untitled7" src="https://github.com/user-attachments/assets/fdcd2db2-e202-473b-abcc-d04397101d1c" />





