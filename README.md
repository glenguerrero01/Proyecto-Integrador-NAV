# Proyecto-Integrador-NAV - Grupo 4
Repositorio proyecto integrador por William Andrade, Glen Guerrero
## *ACTUALIZACIÓN - ENTREGA FINAL* 
### *SLAM*
Esta vez ya se logró obtener el mapa en el que el robot hizo el escaneo, se realizó la construcción de un mapa de ocupación a partir de las lecturas de los sensores de odometría y lidar(/scan). El mapa se representa en log-odds (l = log(p/(1-p))). Inicialmente se encontró que se requería aplicar un offset y una rotación para lograr alineaar la trayectoria con el mapa, igualmente una vez obtenido el mapa se procesa filtrando y modificando para binarizarlo.
| Mapa EKF-SLAM |  Mapa Offset | Mapa ocupación | Mapa procesado 
| :---: | :---: | :---: | :---: |
|<img width="180" src="https://github.com/user-attachments/assets/c086bb65-e7a3-460b-89c4-024ad623bee6" />|<img width="180" src="https://github.com/user-attachments/assets/10da1ef0-581c-451c-99f5-e5f936559df3" />|<img width="150" src="https://github.com/user-attachments/assets/6e97d8b5-4b14-444f-afed-45f6e4bd451c" />|<img width="153" src="https://github.com/user-attachments/assets/d767b7bf-fc1e-443c-a3f8-f88645941418" />

### *PLANIFICACIÓN*

Se utilizó los mismos códigos de la anterior entrega, solamente se actualizó el mapa de ocupación del entorno ya capturado por el robot mediante el rosbag.
| A* |  RRT*  
| :---: | :---: |
|<img width="180" src="https://github.com/user-attachments/assets/36cccbb5-2551-4373-ab67-63f17312ef14" />|<img width="180" src="https://github.com/user-attachments/assets/eee254ed-d1e1-4956-b76f-d6030beb6281" />

### *CONTROL PURE PURSUIT*
Para este control no se realizaron pruebas de ambos controladores, sino que directamente se decidió implementar el controlador Pure Pursuit por su facilidad de programación, buena respuesta y fácil ajuste. Para poder usar el controlador, es importante entender el funcionamiento del planificador, el cual es por coordenadas en rejilla, pero el controlador Pure Pursuit requiere waypoints. Se realizaron varias pruebas diferentes del valor de Ld; en la tabla 5 se presenta las 5 más relevantes. Se realiza la impresión de RMS del error lateral (Error lateral medio) y Error lateral máximo. Para la velocidad angular, velocidad lineal y error lateral se realiza las gráficas respectivas.  
| Ld | 0.25 | 0.3 | 0.4 | 0.5 | 0.8 |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **RMS error lateral [m]** | 0.2311 | 0.2137 | 0.2125 | 0.2128 | 0.2414 |
| **Error lateral MAX [m]** | 0.1045 | 0.1044 | 0.1060 | 0.1086 | 0.1189 |
<img width="300" src="https://github.com/user-attachments/assets/fd3031d6-6961-4c5f-a9d5-adba8e9256e2" />

## *ACTUALIZACIÓN - Parte 3* - Algortimo de planificación de trayectoria
Antes de realizar pruebas del mapa de ocupación obtenido por SLAM, se ha decido probar con un mapa complejo del toolbox de Matlab para inicialmente verificar el funcionamiento de 2 de los algoritmos de planificación trabajado en el curso, El primero correspondiente basado en planificación por rejilla **(A*)** y el otro por muestreo **(RRT*)**.

Para ambos código ser programo una punto inicial y un punto objetivo  o final; Dentro de la simulación se programó la posibilidad de aparecer obstaculos de manera aleatoria, teniendo en cuenta la complejidad del mapa solamente se da la orden de que aparezca un obstaculo al azar. El obstaculo se podrá observar en color un cuadro rojo para ambos planificadores.
| A* Trayectoria 1 |  A* Trayectoria 2 Nueva planificación  
| :---: | :---: |
|<img width="250" src="https://github.com/user-attachments/assets/426cb504-954e-4128-8c89-0c2045fbeead"> |<img width="250" src="https://github.com/user-attachments/assets/c9096b0c-7556-4c9d-8b7b-e6d825750cea">

| RRT* Trayectoria 1 |  Aparece obstáculo |RRT* Trayectoria 2 Nueva planificación
| :---: | :---: | :---: |
|<img width="376" src="https://github.com/user-attachments/assets/e1bcfb5a-2118-407b-bd5a-ab52091cd4ff"> |<img width="376" src="https://github.com/user-attachments/assets/a8837daa-ada3-4889-9a4b-ba0ea023a58b">|<img width="376" src="https://github.com/user-attachments/assets/fe1d491c-9270-4b15-a41f-9916a305eff4">





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
<img width="376" alt="untitled1" src="https://github.com/user-attachments/assets/2ad2ea9f-8508-4c74-9ad5-66e7a306e564" />
<img width="350"  alt="untitled2_1" src="https://github.com/user-attachments/assets/d01a5251-622d-4ca4-80b6-af07b347e97f" />

### *Prueba 2*
<img width="376" alt="untitled3" src="https://github.com/user-attachments/assets/4b8ce86d-dc2b-4408-9409-528488659670" />
<img width="350"  alt="image" src="https://github.com/user-attachments/assets/3fa9951e-c251-45f2-a4cf-a193ddf05cf4" />

### *Prueba 3*
<img width="376" alt="untitled7" src="https://github.com/user-attachments/assets/c3c50f5a-953f-46da-8ee6-e2e43e22eb76" />
<img width="350" alt="image" src="https://github.com/user-attachments/assets/097a60da-6850-483b-861c-7376973107c1" />


## *Parte 1* - Algortimo de localización con filtro EKF
Se ha implementado a la función analyze_bag un filtro EKF aplicado a la odometría obtenida a través del Rosbag; AMCL ya estaba integrado en el código.

<img width="400" height="350" alt="image" src="https://github.com/user-attachments/assets/a80413e9-d241-4ef8-8224-d437ac9c6d69" />
<img width="400" height="350" alt="image" src="https://github.com/user-attachments/assets/841d2fcb-6286-4f42-900a-904b4ec88a89" />



