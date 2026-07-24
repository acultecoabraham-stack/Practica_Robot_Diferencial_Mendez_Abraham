%% =========================================================
% CINEMATICA INVERSA DE UN ROBOT DIFERENCIAL CON ANIMACION Y EXPORTACION
%
% El robot sigue una trayectoria deseada (circular) 
% calculando las velocidades requeridas en cada rueda.
% Guarda la animacion en GIF y las graficas en PNG.
% =========================================================

clear all;
clc;
close all;

%% CREAR CARPETA PARA GUARDAR RESULTADOS
nombre_carpeta = 'Resultados_Cinematica_Inversa';
if ~exist(nombre_carpeta,'dir')
    mkdir(nombre_carpeta);
end

%% TIEMPO DE SIMULACION
ts = 15;                % Tiempo total de simulacion [s]
ti = 0.01;              % Paso de integracion [s]
t = 0:ti:ts;            % Vector de tiempo
N = length(t);          % Numero de muestras

%% PARAMETROS DEL ROBOT
L = 0.18;               % Distancia entre ruedas [m]
r = 0.03;               % Radio de las ruedas [m]

%% CONDICIONES INICIALES (theta inicial = pi/2)
x = zeros(1, N);
y = zeros(1, N);
theta = zeros(1, N);

x(1) = 1.0;             % Posicion inicial en X [m]
y(1) = 0.0;             % Posicion inicial en Y [m]
theta(1) = pi/2;        % Orientacion inicial rotada 90 grados [rad]

%% VECTORES PARA GUARDAR RESULTADOS
u = zeros(1, N);
w = zeros(1, N);
thetaDot_D = zeros(1, N);
thetaDot_I = zeros(1, N);

%% TRAYECTORIA DESEADA MATEMATICA (Ruta Circular)
R_curva = 1.0;          % Radio de la trayectoria [m]
w_c = 0.4;              % Velocidad angular de la trayectoria [rad/s]

% Derivadas de la posicion (Velocidades deseadas en el espacio)
x_punto = -R_curva * w_c * sin(w_c * t); 
y_punto =  R_curva * w_c * cos(w_c * t);
theta_punto = w_c * ones(1, N);

%% BUCLE DE CINEMATICA INVERSA Y METODO DE EULER
for k = 1:N-1
    % 1. Transformacion del espacio de tarea al espacio del robot
    u(k) = x_punto(k)*cos(theta(k)) + y_punto(k)*sin(theta(k));
    w(k) = theta_punto(k);
    
    % 2. Ecuaciones de la Cinematica Inversa (Velocidades de actuadores)
    thetaDot_D(k) = (u(k) + (L/2)*w(k)) / r;
    thetaDot_I(k) = (u(k) - (L/2)*w(k)) / r;
    
    % 3. Integracion numerica (Actualizar posicion del robot)
    x(k+1) = x(k) + ti * (u(k) * cos(theta(k)));
    y(k+1) = y(k) + ti * (u(k) * sin(theta(k)));
    theta(k+1) = theta(k) + ti * w(k);
    
    % Normalizar el angulo entre -pi y pi
    theta(k+1) = atan2(sin(theta(k+1)), cos(theta(k+1)));
end

% Llenar el ultimo valor de los vectores de velocidad para graficar parejo
u(end) = u(end-1); 
w(end) = w(end-1);
thetaDot_D(end) = thetaDot_D(end-1); 
thetaDot_I(end) = thetaDot_I(end-1);

%% =========================================================
% INICIO DE LA ANIMACION Y CREACION DEL GIF
% =========================================================

figura_animacion = figure('Name', 'Animacion Cinematica Inversa');
set(figura_animacion, 'Position', [150, 150, 800, 600]);

% Limites fijos de la grafica
xmin = min(x) - 0.5; xmax = max(x) + 0.5;
ymin = min(y) - 0.5; ymax = max(y) + 0.5;

paso_animacion = 30; % Se dibuja 1 de cada 30 cuadros
indices_animacion = [1:paso_animacion:N, N];

% Nombre del archivo GIF
nombre_gif = fullfile(nombre_carpeta, 'Animacion_Cinematica_Inversa.gif');
k_contador = 1;

for k_anim = indices_animacion
    if ~ishandle(figura_animacion), break; end
    
    cla; % Limpiar el cuadro anterior
    hold on; grid on; axis equal;
    xlim([xmin xmax]); ylim([ymin ymax]);
    
    % A. Dibujar la ruta matematica deseada
    plot(x, y, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);
    
    % B. Dibujar la ruta recorrida
    plot(x(1:k_anim), y(1:k_anim), 'b', 'LineWidth', 3);
    
    % C. Dibujar el robot
    plot(x(k_anim), y(k_anim), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    
    % D. Dibujar la orientacion
    quiver(x(k_anim), y(k_anim), 0.2*cos(theta(k_anim)), 0.2*sin(theta(k_anim)), ...
        'Color', 'k', 'LineWidth', 2.5, 'MaxHeadSize', 2);
        
    % E. Actualizar titulos
    titulo_1 = 'Seguimiento de Trayectoria Circular (Cinematica Inversa)';
    titulo_2 = sprintf('Tiempo: %.2f / %.2f s', t(k_anim), ts);
    titulo_3 = sprintf('Rueda Der: %.1f rad/s | Rueda Izq: %.1f rad/s', ...
        thetaDot_D(k_anim), thetaDot_I(k_anim));
        
    title({titulo_1; ['\bf' titulo_2]; titulo_3}, 'FontSize', 13);
    xlabel('Posicion X [m]'); ylabel('Posicion Y [m]');
    set(gca, 'FontSize', 12);
    
    drawnow; % Renderizar el grafico en vivo

    % --- CAPTURA DE FRAMES PARA EL GIF ---
    frame = getframe(figura_animacion);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if k_contador == 1
        imwrite(imind, cm, nombre_gif, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, nombre_gif, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
    k_contador = k_contador + 1;
end

% Guardar la imagen final de la trayectoria
if ishandle(figura_animacion)
    saveas(figura_animacion, fullfile(nombre_carpeta, 'Trayectoria_Circular_Final.png'));
end

%% =========================================================
% GRAFICAS ESTATICAS PARA TU REPORTE
% =========================================================

% 1. Perfiles de velocidad lineal y angular del robot
figura_vel_robot = figure('Name', 'Velocidades del Robot');
plot(t, u, 'b', 'LineWidth', 3); hold on;
plot(t, w, 'r', 'LineWidth', 3);
grid on;
xlabel('Tiempo [s]'); ylabel('Velocidad');
title('Velocidad Lineal (u) y Angular (w) del Robot');
legend('u [m/s]', 'w [rad/s]', 'Location', 'best');
set(gca, 'FontSize', 14);
% Guardar figura
saveas(figura_vel_robot, fullfile(nombre_carpeta, 'Velocidades_Robot.png'));

% 2. Perfiles de velocidad requeridos en las ruedas
figura_vel_ruedas = figure('Name', 'Velocidades de las Ruedas');
plot(t, thetaDot_D, 'b', 'LineWidth', 3); hold on;
plot(t, thetaDot_I, 'r--', 'LineWidth', 3);
grid on;
xlabel('Tiempo [s]'); ylabel('Velocidad Angular [rad/s]');
title('Velocidades Exigidas a los Actuadores');
legend('\theta_D (Rueda Derecha)', '\theta_I (Rueda Izquierda)', 'Location', 'best');
set(gca, 'FontSize', 14);
% Guardar figura
saveas(figura_vel_ruedas, fullfile(nombre_carpeta, 'Velocidades_Ruedas.png'));

fprintf('\n¡Proceso exitoso! Todas las imágenes y el GIF se guardaron en la carpeta: %s\n', nombre_carpeta);