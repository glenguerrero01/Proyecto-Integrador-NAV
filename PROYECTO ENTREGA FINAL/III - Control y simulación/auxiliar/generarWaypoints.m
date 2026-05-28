 % Esta función convierte la ruta de la rejilla A* a waypoints en metros para Pure Pursuit
function waypoints = generarWaypoints(ruta_grid, cell_size, escala)
   
    % ruta_grid: Matriz Nx2 con la ruta del A* [fila(Y), columna(X)]
    % cell_size: Tamaño de la celda usado en la matriz 
    % escala: Factor de conversión de metros a pixeles (ej. 10)

    % ruta_grid(:, 1) = filas = Coordenada Y
    % ruta_grid(:, 2) = columnas = Coordenada X
    Y_grid = ruta_grid(:, 1);
    X_grid = ruta_grid(:, 2);
    
    % Restamos 0.5 para apuntar al centro de la celda y convertimos a metros
    X_metros = ((X_grid - 0.5) * cell_size) / escala;
    Y_metros = ((Y_grid - 0.5) * cell_size) / escala;
    
    % Ensamblamos los waypoints [X, Y]
    waypoints = [X_metros, Y_metros];
end