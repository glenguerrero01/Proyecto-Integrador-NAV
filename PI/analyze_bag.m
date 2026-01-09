function analyze_bag(bagFile)
% Script básico para analizar un rosbag de MIR
%
% Uso:
%   analyze_mir_bag('ruta/al/rosbag.bag')

%----------------------------
% 0) Cargar bag
%----------------------------
if nargin < 1
    [f, p] = uigetfile('*.bag', 'Selecciona un rosbag');
    if isequal(f,0)
        disp('No se ha seleccionado archivo.');
        return;
    end
    bagFile = fullfile(p, f);
end

fprintf('Cargando rosbag: %s\n', bagFile);
bag = rosbag(bagFile);

fprintf('\nTopics disponibles:\n');
disp(bag.AvailableTopics.Properties.RowNames);

% Helper para comprobar si existe un topic
hasTopic = @(name) any(strcmp(name, bag.AvailableTopics.Properties.RowNames));

% Helper para tiempo (segundos)
timeFromHeader = @(h) double(h.Stamp.Sec) + double(h.Stamp.Nsec)*1e-9;

% Helper para yaw desde cuaternión
quatToYaw = @(q) atan2( ...
    2*(q.W*q.Z + q.X*q.Y), ...
    1 - 2*(q.Y^2 + q.Z^2));

%----------------------------
% 1) ODOMETRÍA (/odom)
%----------------------------
odom = [];
if hasTopic('/odom')
    bagOdom = select(bag, 'Topic', '/odom');
    odomMsgs = readMessages(bagOdom, 'DataFormat', 'struct');

    n = numel(odomMsgs);
    odom.t   = zeros(n,1);
    odom.x   = zeros(n,1);
    odom.y   = zeros(n,1);
    odom.yaw = zeros(n,1);

    for k = 1:n
        m = odomMsgs{k};
        odom.t(k) = timeFromHeader(m.Header);
        odom.x(k) = m.Pose.Pose.Position.X;
        odom.y(k) = m.Pose.Pose.Position.Y;

        q = m.Pose.Pose.Orientation;
        odom.yaw(k) = quatToYaw(q);
    end
    fprintf('Leídos %d mensajes de /odom\n', n);
else
    warning('No se ha encontrado /odom en el bag.');
end

%----------------------------
% 2) GROUND TRUTH (/base_pose_ground_truth)
%----------------------------
gt = [];
if hasTopic('/base_pose_ground_truth')
    bagGT = select(bag, 'Topic', '/base_pose_ground_truth');
    gtMsgs = readMessages(bagGT, 'DataFormat', 'struct');

    n = numel(gtMsgs);
    gt.t   = zeros(n,1);
    gt.x   = zeros(n,1);
    gt.y   = zeros(n,1);
    gt.yaw = zeros(n,1);

    for k = 1:n
        m = gtMsgs{k};
        gt.t(k) = timeFromHeader(m.Header);

        % OJO: aquí es Pose, no pose
        gt.x(k) = m.Pose.Pose.Position.X;
        gt.y(k) = m.Pose.Pose.Position.Y;

        q = m.Pose.Pose.Orientation;
        gt.yaw(k) = quatToYaw(q);
    end
    fprintf('Leídos %d mensajes de /base_pose_ground_truth\n', n);
else
    fprintf('No hay /base_pose_ground_truth (no se pintará GT).\n');
end


%----------------------------
% 3) AMCL (/amcl_pose)
%----------------------------
amcl = [];
if hasTopic('/amcl_pose') %Si existe en el ros bag
    bagAmcl = select(bag, 'Topic', '/amcl_pose');
    amclMsgs = readMessages(bagAmcl, 'DataFormat', 'struct');

    n = numel(amclMsgs);
    amcl.t   = zeros(n,1);
    amcl.x   = zeros(n,1);
    amcl.y   = zeros(n,1);
    amcl.yaw = zeros(n,1);

    for k = 1:n
        m = amclMsgs{k};
        amcl.t(k) = timeFromHeader(m.Header);
        amcl.x(k) = m.Pose.Pose.Position.X;
        amcl.y(k) = m.Pose.Pose.Position.Y;

        q = m.Pose.Pose.Orientation;
        amcl.yaw(k) = quatToYaw(q);
    end
    if ~isempty(amcl)
        % Restar el primer punto para que empiece en el origen (opcional, para comparar formas)
        amcl.x_norm = amcl.x - amcl.x(1);
        amcl.y_norm = amcl.y - amcl.y(1);

        % Si quieres compararlas en el mismo gráfico partiendo de donde empezó el Ground Truth:
        % amcl.x = amcl.x - (amcl.x(1) - gt.x(1));
    end
    fprintf('Leídos %d mensajes de /amcl_pose\n', n);
else
    fprintf('No hay /amcl_pose (no se pintará AMCL).\n');
end

%  SCAN (/scan)
%----------------------------
scan = [];
if hasTopic('/scan')
    bagScan = select(bag, 'Topic', '/scan');
    scanMsgs = readMessages(bagScan, 'DataFormat', 'struct');

    n = numel(scanMsgs);
    scan.t = zeros(n,1);
    scan.ranges = cell(n,1);

    for k = 1:n
        m = scanMsgs{k};
        scan.t(k) = timeFromHeader(m.Header);
        scan.ranges{k} = m.Ranges;
    end
    fprintf('Leídos %d mensajes de /scan\n', n);
else
    fprintf('No hay /scan (no se pintará Scan).\n');
end



% IMU DATA (/imu_data)
%----------------------------
imu = [];
if hasTopic('/imu_data')
    bagImu = select(bag, 'Topic', '/imu_data');
    imuMsgs = readMessages(bagImu, 'DataFormat', 'struct');

    n = numel(imuMsgs);
    imu.t = zeros(n,1);
    imu.orientation = zeros(n,4); % Cuaterniones

    for k = 1:n
        m = imuMsgs{k};
        imu.t(k) = timeFromHeader(m.Header);
        imu.orientation(k, :) = [m.Orientation.W, m.Orientation.X, m.Orientation.Y, m.Orientation.Z];
    end
    fprintf('Leídos %d mensajes de /imu_data\n', n);
else
    fprintf('No hay /imu_data (no se pintará IMU).\n');
end


%Filtro ekf aplicado a la odometría
fprintf('Aplicando EKF a la odometría...\n');

n_samples = length(odom.t);

% --- Inicialización del Estado [x; y; theta] ---
% El estado inicial es la primera lectura de la odometría
x_est = [odom.x(1); odom.y(1); odom.yaw(1)];
%x_est = x_est + [0.2; -0.2; 0.05]; %Error inicial según la actividad 1 que se hizo


% P: Covarianza del error inicial (Incertidumbre inicial)
P = eye(3) * 0.01;


% Ruido de sensores (GPS y IMU)
Q_gps = diag([0.9, 0.9]).^2;  % Ruido en posición (metros^2)
Q_imu = deg2rad(20)^2;        % Ruido en orientación (radianes^2)
Q = blkdiag(Q_gps, Q_imu);    % Matriz de covarianza combinada del sensor
% Ajustando el ruido del proceso para que refleje mejor la odometría
%Q = diag([0.01, 0.01, 0.1]);
%Q = diag([0.9, 0.9, 0.1]).^2;

% R: Ruido de la Medición (¿Qué tan malo es el sensor?)
% Valores altos = confiamos poco en el sensor -> más suavizad
%R = diag([0.1, 0.1, 0.05]);

% Ruido del modelo de movimiento (odometría)
R = diag([0.2, 0.2, 0.1]).^2;

% Reservar memoria para guardar resultados
ekf_x = zeros(n_samples, 1);
ekf_y = zeros(n_samples, 1);
ekf_yaw = zeros(n_samples, 1);

% Guardar el primer punto
ekf_x(1) = x_est(1);
ekf_y(1) = x_est(2);
ekf_yaw(1) = x_est(3);

% --- Bucle EKF ---
for k = 2:n_samples
    dt = odom.t(k) - odom.t(k-1);
    if dt <= 0, dt = 0.01; end % Evitar división por cero o tiempos negativos

    % 1. PREDICCIÓN (Modelo de Movimiento)
    % Como no estamos leyendo velocidades (Twist), usamos el estado anterior
    % como mejor predicción (Random Walk assumption para posición).
    x_pred = x_est;

    % Jacobiano del modelo (Identidad en este caso simple)
    F = eye(3);

    % Predicción de la covarianza
    P_pred = F * P * F' + Q;

    % 2. ACTUALIZACIÓN (Corrección con la medición)
    % Vector de medición Z actual
    z = [odom.x(k); odom.y(k); odom.yaw(k)];

    % Calcular el residuo (Innovación)
    y_residual = z - x_pred;

    % IMPORTANTE: Normalizar el ángulo del residuo entre -pi y pi
    % Esto evita que el filtro "explote" cuando el robot gira de 3.14 a -3.14
    y_residual(3) = atan2(sin(y_residual(3)), cos(y_residual(3)));

    % Matriz H (Relación estado-medición, Identidad aquí)
    H = eye(3);

    % Cálculo de la Ganancia de Kalman (K)
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;

    % Actualizar el estado con la ganancia óptima
    x_est = x_pred + K * y_residual;

    % Actualizar la covarianza P
    P = (eye(3) - K * H) * P_pred;

    % Guardar datos filtrados
    ekf_x(k) = x_est(1);
    ekf_y(k) = x_est(2);
    ekf_yaw(k) = x_est(3);
end
ekf.x = ekf_x;
ekf.y = ekf_y;
ekf.t = odom.t;
ekf.yaw=ekf_yaw;



%----------------------------
% 4) PLOTS BÁSICOS
%----------------------------

% 4.1 Trayectoria XY
figure('Name','Trayectorias','NumberTitle','off');
hold on; grid on; axis equal;
if ~isempty(gt)
    plot(gt.x, gt.y, 'k-', 'DisplayName', 'Ground truth');
end
if ~isempty(odom)
    plot(odom.x, odom.y, 'b-', 'DisplayName', 'Odom', 'LineWidth', 2); %b-- para punteada
end
if ~isempty(amcl)
    plot(amcl.x, amcl.y, 'r-', 'DisplayName', 'AMCL');
end


if ~isempty(x_est)
    plot(ekf.x, ekf.y, 'g-', 'DisplayName', 'EKF', 'LineWidth', 0.5);
end



xlabel('x [m]');
ylabel('y [m]');
title('Trayectorias 2D');
legend('Location','best');
hold off;

% 4.2 Yaw vs tiempo (si hay odom y/o amcl)
if ~isempty(odom) || ~isempty(amcl) || ~isempty(gt) || ~isempty(est)
    figure('Name','Yaw vs tiempo','NumberTitle','off');
    hold on; grid on;
    if ~isempty(odom)
        plot(odom.t - odom.t(1), odom.yaw, 'b-', 'DisplayName', 'Odom');
    end
    if ~isempty(amcl)
        plot(amcl.t - amcl.t(1), amcl.yaw, 'r-', 'DisplayName', 'AMCL');
    end
    if ~isempty(gt)
        plot(gt.t - gt.t(1), gt.yaw, 'k-', 'DisplayName', 'GT');
    end

    if ~isempty(x_est)
        plot(ekf.t - ekf.t(1), ekf.yaw, 'g-', 'DisplayName', 'EKF'); %ekf_yaw
    end


    xlabel('t [s]');
    ylabel('yaw [rad]');
    title('Orientación (yaw) vs tiempo');
    legend('Location','best');
    hold off;
end


