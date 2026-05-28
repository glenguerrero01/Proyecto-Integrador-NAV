function map = dibujarMapaV2(q_start, q_goal, rutaImagen)
    % 1. Cargar la imagen desde el ordenador
    img = imread(rutaImagen);
    
    % 2. Si la imagen es a color (RGB), convertirla a escala de grises
    if size(img, 3) == 3
        img = rgb2gray(img);
    end
    
    % 3. Binarizar la imagen
    % imbinarize asigna 1 (blanco) al fondo y 0 (negro) a las líneas oscuras.
    mapa_binario = imbinarize(img);
    
    % 4. Invertir la matriz para que los obstáculos sean 1 (true) y el área libre 0 (false)
    map = ~mapa_binario;
    
    % Obtener las dimensiones del mapa
    [rows, cols] = size(map);
    
    % Dibujar la figura
    figure;
    imshow(map, 'XData', [0 cols], 'YData', [0 rows], 'InitialMagnification', 'fit');
    
    % Colormap: el valor 0 será blanco [1 1 1] y el valor 1 será negro [0 0 0]
    colormap([1 1 1; 0 0 0]);
    hold on;
    
    % Dibujar los puntos de inicio y meta
    plot(q_start(1), q_start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(q_goal(1), q_goal(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    
    title('Entorno');
    xlabel('X'); ylabel('Y');
    axis([0 cols 0 rows]);
    axis equal;
end