function plot_scan(bagFile)

    if nargin < 1
        [f, p] = uigetfile('*.bag', 'Selecciona un rosbag');
        if isequal(f,0)
            disp('No se ha seleccionado archivo.');
            return;
        end
        bagFile = fullfile(p, f);
    end

    bag = rosbag(bagFile);

    % Comprobar que existe /scan
    topics = bag.AvailableTopics.Properties.RowNames;
    if ~any(strcmp('/scan', topics))
        error('El bag no tiene el topic /scan');
    end

    % Leer todos los mensajes de /scan
    bagScan  = select(bag, 'Topic', '/scan');
    scanMsgs = readMessages(bagScan, 'DataFormat', 'struct');

    % Coger un scan "representativo" (el del medio)
    idx = round(numel(scanMsgs)/2);
    idx = 1;
    s   = scanMsgs{idx};

    fprintf('Scan %d de %d\n', idx, numel(scanMsgs));
    fprintf('RangeMin = %.3f, RangeMax = %.3f\n', s.RangeMin, s.RangeMax);

    % Pasar a double y construir ángulos
    ranges = double(s.Ranges(:));  % columna
    N      = numel(ranges);
    angles = double(s.AngleMin + (0:N-1)' * s.AngleIncrement);

    % FILTRO: solo puntos válidos que ven obstáculo
    mask = isfinite(ranges) & ...
           (ranges > s.RangeMin + 1e-3) & ...
           (ranges < s.RangeMax - 1e-3);

    ranges_obs = ranges(mask);
    angles_obs = angles(mask);

    x = ranges_obs .* cos(angles_obs);
    y = ranges_obs .* sin(angles_obs);

    figure('Name','Scan láser (solo obstáculos)','NumberTitle','off');
    scatter(x, y, 10, 'filled');   % puntos sueltos, sin unir
    axis equal; grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    title(sprintf('Topic /scan – %d puntos de obstáculo', numel(x)));

  % -- Implementación de LidarSLAM% 
% Parámetros ajustables
mapResolution = 15;     % Celdas por metro
maxLidarRange = 25;     % Dist máx lidar en Metros (Se usa un poco menor según recomendación matlab(Referencias))
skipStep      = 40;      % Procesar 1 de cada X mensajes (para velocidad)

% Inicializar algoritmo SLAM
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 3; %8
%slamAlg.MapResolution = mapResolution;

rangoInicial = 20;
rangoFinal = 100; %45

%% 3. Bucle de Procesamiento

for i=rangoInicial:rangoFinal %Desde 15 a 30
    s = scanMsgs{i};
    ranges = double(s.Ranges(:)); 
    ranges(isnan(ranges)) = Inf;

    N = numel(ranges);
    % Cálculo de ángulos
    angles = double(s.AngleMin + (0:N-1)' * s.AngleIncrement);
    scanObj = lidarScan(ranges, angles);    % objeto para el mapeo

    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scanObj); %scanObj
    if isScanAccepted
        fprintf('Added scan %d \n', i);
    end

end

figure;
show(slamAlg);
title({sprintf('LidarSLAM - Map Res: %d, I Range: %d, F Range: %d,   Threshold: %d,  Radius: %d', ...
    mapResolution, rangoInicial, rangoFinal, slamAlg.LoopClosureThreshold, slamAlg.LoopClosureSearchRadius)});


    %% 4. Generación y Visualización del Mapa
disp('Construyendo mapa de ocupación...');

% Obtener los escaneos y las poses optimizadas tras el SLAM
[scans, optimizedPoses] = scansAndPoses(slamAlg);

map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

% Visualización Final
figure();
show(map);
hold on

show(slamAlg.PoseGraph, 'IDs', 'off');
hold off

title('Mapa de Ocupación (Occupancy Grid)');
xlabel('X [m]');
ylabel('Y [m]');
axis equal;

end
