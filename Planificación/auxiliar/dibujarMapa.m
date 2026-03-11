function map = dibujarMapa(q_start, q_goal)
    load exampleMaps.mat complexMap
    map = binaryOccupancyMap(complexMap);

    if isa(map, 'binaryOccupancyMap')
        map = occupancyMatrix(map);
    end
    map = map > 0;
    [rows, cols] = size(map);

    figure;
    imshow(map, 'XData', [0 cols], 'YData', [0 rows], 'InitialMagnification', 'fit');
    colormap([1 1 1; 0 0 0]);
    hold on;

    plot(q_start(1), q_start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(q_goal(1), q_goal(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);

    title('Entorno');
    xlabel('X'); ylabel('Y');
    axis([0 cols 0 rows]);
    axis equal;
end
