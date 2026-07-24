%% =========================================================
% CONTROL CINEMATICO DE UN ROBOT DIFERENCIAL CON ANIMACION Y EXPORTACION
% Comparacion: Modelo ideal (sin saturacion) vs. Modelo real (saturado)
% Guarda graficas en HD usando el comando print (Compatible con versiones anteriores)
% =========================================================
clear all;
clc;
close all;

%% CREAR CARPETA PARA GUARDAR RESULTADOS
nombre_carpeta = 'Resultados_Control_Cinematico';
if ~exist(nombre_carpeta,'dir')
    mkdir(nombre_carpeta);
end

%% TIEMPO DE SIMULACION
ts = 15;              % Tiempo total de simulacion [s]
ti = 0.01;            % Paso de integracion [s]
t = 0:ti:ts;          % Vector de tiempo
N = length(t);        % Numero de muestras

%% PARAMETROS DEL ROBOT
L = 0.18;             % Distancia entre ruedas [m]
r = 0.03;             % Radio de las ruedas [m]
thetaDot_max = 20;    % Velocidad angular maxima de actuadores [rad/s]

%% POSICION DESEADA (META)
xd = 5;               % Posicion deseada en X [m]
yd = 5;               % Posicion deseada en Y [m]

%% GANANCIAS DEL CONTROLADOR
k_rho = 0.3;          % Ganancia de avance
k_theta = 4;          % Ganancia de giro

%% CONDICIONES INICIALES
% Robot 1: Sin saturacion (Ideal)
x_sin = zeros(1,N); y_sin = zeros(1,N); theta_sin = zeros(1,N);
% Robot 2: Con saturacion proporcional (Real)
x_sat = zeros(1,N); y_sat = zeros(1,N); theta_sat = zeros(1,N);

%% VARIABLES DE ESTADO Y CONTROL
rho_sin = zeros(1,N); e_theta_sin = zeros(1,N);
u_sin = zeros(1,N); w_sin = zeros(1,N);
thetaDot_D_sin = zeros(1,N); thetaDot_I_sin = zeros(1,N);

rho_sat = zeros(1,N); e_theta_sat = zeros(1,N);
u_sat = zeros(1,N); w_sat = zeros(1,N);
thetaDot_D_sat = zeros(1,N); thetaDot_I_sat = zeros(1,N);
alpha = ones(1,N);

%% BUCLE DE SIMULACION PRINCIPAL (CALCULOS)
for k = 1:N-1
    % =====================================================
    % 1. ROBOT IDEAL (SIN SATURACION)
    % =====================================================
    ex_sin = xd - x_sin(k);
    ey_sin = yd - y_sin(k);
    rho_sin(k) = sqrt(ex_sin^2 + ey_sin^2);
    theta_d_sin = atan2(ey_sin,ex_sin);
    e_theta_sin(k) = atan2(sin(theta_d_sin-theta_sin(k)), cos(theta_d_sin-theta_sin(k)));
    
    % Ley de control
    u_sin(k) = k_rho*rho_sin(k);
    w_sin(k) = k_theta*e_theta_sin(k);
    
    % Condicion de llegada
    if rho_sin(k) < 0.03
        u_sin(k) = 0; w_sin(k) = 0;
    end
    % Cinematica Inversa Ideal
    thetaDot_D_sin(k) = (u_sin(k) + (L/2)*w_sin(k))/r;
    thetaDot_I_sin(k) = (u_sin(k) - (L/2)*w_sin(k))/r;
    
    % Integracion (Euler)
    x_sin(k+1) = x_sin(k) + ti*u_sin(k)*cos(theta_sin(k));
    y_sin(k+1) = y_sin(k) + ti*u_sin(k)*sin(theta_sin(k));
    theta_sin(k+1) = theta_sin(k) + ti*w_sin(k);
    theta_sin(k+1) = atan2(sin(theta_sin(k+1)), cos(theta_sin(k+1)));
    
    % =====================================================
    % 2. ROBOT REAL (CON SATURACION PROPORCIONAL)
    % =====================================================
    ex_sat = xd - x_sat(k);
    ey_sat = yd - y_sat(k);
    rho_sat(k) = sqrt(ex_sat^2 + ey_sat^2);
    theta_d_sat = atan2(ey_sat,ex_sat);
    e_theta_sat(k) = atan2(sin(theta_d_sat-theta_sat(k)), cos(theta_d_sat-theta_sat(k)));
    
    % Ley de control teorica
    u_calculada = k_rho*rho_sat(k);
    w_calculada = k_theta*e_theta_sat(k);
    
    if rho_sat(k) < 0.03
        u_calculada = 0; w_calculada = 0;
    end
    % Cinematica Inversa (Velocidades requeridas)
    thetaDot_D_calculada = (u_calculada + (L/2)*w_calculada)/r;
    thetaDot_I_calculada = (u_calculada - (L/2)*w_calculada)/r;
    
    % Verificar limite fisico
    valor_maximo = max(abs([thetaDot_D_calculada, thetaDot_I_calculada]));
    
    if valor_maximo > thetaDot_max
        % Escalamiento proporcional (Factor Alpha)
        alpha(k) = thetaDot_max/valor_maximo;
        thetaDot_D_sat(k) = alpha(k)*thetaDot_D_calculada;
        thetaDot_I_sat(k) = alpha(k)*thetaDot_I_calculada;
    else
        alpha(k) = 1;
        thetaDot_D_sat(k) = thetaDot_D_calculada;
        thetaDot_I_sat(k) = thetaDot_I_calculada;
    end
    % Recalcular velocidades lineales y angulares reales (Cinematica Directa)
    u_sat(k) = r*(thetaDot_D_sat(k) + thetaDot_I_sat(k))/2;
    w_sat(k) = r*(thetaDot_D_sat(k) - thetaDot_I_sat(k))/L;
    
    % Integracion (Euler)
    x_sat(k+1) = x_sat(k) + ti*u_sat(k)*cos(theta_sat(k));
    y_sat(k+1) = y_sat(k) + ti*u_sat(k)*sin(theta_sat(k));
    theta_sat(k+1) = theta_sat(k) + ti*w_sat(k);
    theta_sat(k+1) = atan2(sin(theta_sat(k+1)), cos(theta_sat(k+1)));
end
% Completar ultimos valores
rho_sin(end) = sqrt((xd-x_sin(end))^2 + (yd-y_sin(end))^2);
rho_sat(end) = sqrt((xd-x_sat(end))^2 + (yd-y_sat(end))^2);

%% =========================================================
% INICIO DE LA ANIMACION (CARRERA ENTRE ROBOTS)
% =========================================================
figura_animacion = figure('Name', 'Animacion: Control Cinematico');
set(figura_animacion, 'Position', [150, 150, 900, 600]);

margen = 1;
xmin = min([0, xd]) - margen; xmax = max([0, xd]) + margen;
ymin = min([0, yd]) - margen; ymax = max([0, yd]) + margen;

nombre_gif = fullfile(nombre_carpeta, 'Carrera_Control_Cinematico.gif');
k_contador = 1;

paso_animacion = 40; 
indices_animacion = [1:paso_animacion:N, N];

for k_anim = indices_animacion
    if ~ishandle(figura_animacion), break; end
    
    cla; hold on; grid on; axis equal;
    xlim([xmin xmax]); ylim([ymin ymax]);
    
    plot(xd, yd, 'kp', 'MarkerSize', 20, 'LineWidth', 2, 'MarkerFaceColor', 'y');
    plot(x_sin(1:k_anim), y_sin(1:k_anim), 'b', 'LineWidth', 2.5); 
    plot(x_sat(1:k_anim), y_sat(1:k_anim), 'r--', 'LineWidth', 3); 
    plot(x_sin(k_anim), y_sin(k_anim), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    plot(x_sat(k_anim), y_sat(k_anim), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    titulo_1 = 'Control Cinematico: Carrera hacia la meta';
    titulo_2 = sprintf('Tiempo de simulacion: %.2f / %.2f s', t(k_anim), ts);
    titulo_3 = sprintf('Distancia a la meta -> Ideal: %.2f m | Real (Saturado): %.2f m', rho_sin(k_anim), rho_sat(k_anim));
    
    title({titulo_1; ['\bf' titulo_2]; titulo_3}, 'FontSize', 13);
    xlabel('Posicion X [m]'); ylabel('Posicion Y [m]');
    legend('Meta', 'Trayectoria Ideal', 'Trayectoria Real', 'Robot Ideal', 'Robot Real', 'Location', 'best');
    set(gca, 'FontSize', 12);
    
    drawnow;
    
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

%% =========================================================
% GRAFICAS ESTATICAS PARA TU REPORTE (CALIDAD HD CON PRINT)
% =========================================================

% FIGURA 2: COMPARACION DE TRAYECTORIA FINAL
figura_trayectoria = figure('Name', 'Trayectoria Final Comparativa');
plot(x_sin, y_sin, 'b', 'LineWidth', 3); hold on;
plot(x_sat, y_sat, 'r--', 'LineWidth', 3);
plot(0, 0, 'go', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'g');
plot(xd, yd, 'kp', 'MarkerSize', 18, 'LineWidth', 2, 'MarkerFaceColor', 'y');
xlabel('Posicion X [m]'); ylabel('Posicion Y [m]');
title('Control: Ideal (Sin limites) vs. Real (Saturacion Proporcional)');
legend('Ruta Ideal', 'Ruta Real', 'Inicio', 'Meta', 'Location', 'best');
grid on; axis equal; set(gca, 'FontSize', 14);

% Guardar imagen en HD con PRINT
print(figura_trayectoria, fullfile(nombre_carpeta, 'Trayectoria_Comparativa_Control.png'), '-dpng', '-r300');

% FIGURA 3: COMPARACION DE DISTANCIA A LA META (RHO)
figura_error = figure('Name', 'Evolucion del Error de Distancia');
plot(t, rho_sin, 'b', 'LineWidth', 3); hold on;
plot(t, rho_sat, 'r--', 'LineWidth', 3);
xlabel('Tiempo [s]'); ylabel('Distancia al objetivo \rho [m]');
title('Reduccion del error de distancia \rho en el tiempo');
legend('Robot Ideal', 'Robot Real', 'Location', 'best');
grid on; set(gca, 'FontSize', 14);

% Guardar imagen en HD con PRINT
print(figura_error, fullfile(nombre_carpeta, 'Error_Distancia_Rho.png'), '-dpng', '-r300');

% FIGURA 4: VELOCIDADES DE ACTUADORES (EVIDENCIA DE SATURACION)
figura_velocidades = figure('Name', 'Velocidades de las Ruedas');
subplot(2,1,1);
plot(t, thetaDot_D_sin, 'b', 'LineWidth', 2); hold on;
plot(t, thetaDot_D_sat, 'r', 'LineWidth', 2);
yline(thetaDot_max, 'k--', 'Limite Superior', 'LineWidth', 2);
yline(-thetaDot_max, 'k--', 'Limite Inferior', 'LineWidth', 2);
grid on; ylabel('Velocidad \theta_D [rad/s]'); title('Velocidad Rueda Derecha');
legend('Exigida (Ideal)', 'Aplicada (Saturada)', 'Location', 'best');

subplot(2,1,2);
plot(t, thetaDot_I_sin, 'b', 'LineWidth', 2); hold on;
plot(t, thetaDot_I_sat, 'r', 'LineWidth', 2);
yline(thetaDot_max, 'k--', 'Limite Superior', 'LineWidth', 2);
yline(-thetaDot_max, 'k--', 'Limite Inferior', 'LineWidth', 2);
grid on; xlabel('Tiempo [s]'); ylabel('Velocidad \theta_I [rad/s]'); title('Velocidad Rueda Izquierda');

% Guardar imagen en HD con PRINT
print(figura_velocidades, fullfile(nombre_carpeta, 'Saturacion_Actuadores.png'), '-dpng', '-r300');

fprintf('\n¡Todo listo! El GIF y las 3 gráficas finales en HD se guardaron en la carpeta: %s\n', nombre_carpeta);