%% =========================================================
% CINEMATICA DIRECTA DE UN ROBOT DIFERENCIAL CON ANIMACION Y EXPORTACION
%
% Se analizan y animan 8 combinaciones de velocidades, 
% guardando las 8 imagenes estaticas y 8 GIFs animados (uno por caso).
% =========================================================
clear all;
clc;
close all;

%% TIEMPO DE SIMULACION
ts = 30;                 % Tiempo total de simulacion [s]
ti = 0.01;               % Paso de integracion [s]
t = 0:ti:ts;             % Vector de tiempo
N = length(t);           % Numero de muestras

%% PARAMETROS DEL ROBOT
r = 0.05;                % Radio de las ruedas [m]
L = 0.22;                % Distancia entre ruedas [m]
thetaDot_max = 20;       % Velocidad maxima de rueda [rad/s]

%% CONDICIONES INICIALES
x0 = 0;                  % Posicion inicial en X [m]
y0 = 0;                  % Posicion inicial en Y [m]
theta0 = 0;              % Orientacion inicial [rad]

%% VELOCIDADES ANGULARES ASIGNADAS (8 CASOS)
velocidades_ruedas = [
     10,  10;            % Caso 1
    -10, -10;            % Caso 2
     12,   6;            % Caso 3
      6,  12;            % Caso 4
     10, -10;            % Caso 5
     10,   0;            % Caso 6
     10,   8;            % Caso 7
     10,   2             % Caso 8
];

numero_casos = size(velocidades_ruedas,1);

%% INICIALIZACION DE VARIABLES DE RESULTADOS
u_resultado = zeros(numero_casos,1);
w_resultado = zeros(numero_casos,1);
radio_giro = zeros(numero_casos,1);
x_final = zeros(numero_casos,1);
y_final = zeros(numero_casos,1);
tipo_movimiento = strings(numero_casos,1);

%% MATRICES PARA GUARDAR TODAS LAS TRAYECTORIAS COMPLETAS
X_todas = zeros(numero_casos,N);
Y_todas = zeros(numero_casos,N);
Theta_todas = zeros(numero_casos,N);

%% CREAR CARPETA PARA GUARDAR RESULTADOS
nombre_carpeta = 'Resultados_con_animacion';
if ~exist(nombre_carpeta,'dir')
    mkdir(nombre_carpeta);
end

% Preparar figura para animaciones
figura_animacion = figure('Name', 'Animacion Casos Cinematica Directa', 'NumberTitle', 'off');
set(figura_animacion, 'Position', [100, 100, 900, 700]); 

%% BUCLE PRINCIPAL DE LOS OCHO CASOS
for caso = 1:numero_casos
    
    % 1. Extraer velocidades
    theta_punto_D = velocidades_ruedas(caso,1);
    theta_punto_I = velocidades_ruedas(caso,2);
    
    % VERIFICACION DEL LIMITE FISICO
    if abs(theta_punto_D) > thetaDot_max || abs(theta_punto_I) > thetaDot_max
        warning('El caso %d supera la velocidad maxima permitida.',caso);
    end
    
    % 2. Calculos cinematicos basicos
    vD = r*theta_punto_D;
    vI = r*theta_punto_I;
    u = (vD + vI)/2;
    w = (vD - vI)/L;
    
    if abs(w) < 1e-10
        Rg = Inf;
    else
        Rg = u/w;
    end
    
    % 3. Guardar parametros calculados
    u_resultado(caso) = u;
    w_resultado(caso) = w;
    radio_giro(caso) = Rg;
    
    % 4. Identificar movimiento para el reporte
    movimiento = "No identificado";
    if abs(u) < 1e-10 && abs(w) < 1e-10
        movimiento = "Detenido";
    elseif abs(w) < 1e-10 && u > 0
        movimiento = "Rectilineo Adelante";
    elseif abs(w) < 1e-10 && u < 0
        movimiento = "Rectilineo Atras";
    elseif abs(u) < 1e-10 && w > 0
        movimiento = "Giro Eje Izq";
    elseif abs(u) < 1e-10 && w < 0
        movimiento = "Giro Eje Der";
    elseif w > 0 && abs(Rg) <= 0.20
        movimiento = "Curva Cerrada Izq";
    elseif w > 0
        movimiento = "Curva Izq";
    elseif w < 0 && abs(Rg) <= 0.20
        movimiento = "Curva Cerrada Der";
    elseif w < 0
        movimiento = "Curva Der";
    end
    tipo_movimiento(caso) = movimiento;
    
    % 5. Pre-calcular la trayectoria completa (Euler)
    x = zeros(1,N); y = zeros(1,N); theta = zeros(1,N);
    x(1) = x0; y(1) = y0; theta(1) = theta0;
    
    for k = 1:N-1
        x(k+1) = x(k) + ti * (u * cos(theta(k)));
        y(k+1) = y(k) + ti * (u * sin(theta(k)));
        theta(k+1) = theta(k) + ti * w;
    end
    
    % Guardar para la comparativa final y resultados
    X_todas(caso,:) = x;
    Y_todas(caso,:) = y;
    Theta_todas(caso,:) = theta;
    x_final(caso) = x(end);
    y_final(caso) = y(end);
    
    % =========================================================
    % ANIMACION PASO A PASO PARA EL CASO ACTUAL
    % =========================================================
    margen = 0.2;
    xmin = min(x) - margen; xmax = max(x) + margen;
    ymin = min(y) - margen; ymax = max(y) + margen;
    
    if xmin == xmax, xmin = xmin-0.5; xmax = xmax+0.5; end
    if ymin == ymax, ymin = ymin-0.5; ymax = ymax+0.5; end
    
    paso_animacion = 50; 
    indices_animacion = [1:paso_animacion:N, N]; 
    
    % Definir el nombre del archivo GIF dinámicamente para cada caso
    nombre_gif = fullfile(nombre_carpeta, sprintf('Animacion_Caso_%d.gif', caso));

    k_contador = 1;
    for k_anim = indices_animacion
        if ~ishandle(figura_animacion), break; end
        
        figure(figura_animacion);
        cla; 
        hold on; grid on; axis equal;
        xlim([xmin xmax]); ylim([ymin ymax]);
        
        % A. Trayectoria de fondo
        plot(x, y, 'Color', [0.8 0.8 0.8], 'LineWidth', 1, 'LineStyle', '--');
        
        % B. Inicio y final
        plot(x0, y0, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        plot(x(end), y(end), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        
        % C. Trayectoria recorrida
        plot(x(1:k_anim), y(1:k_anim), 'b', 'LineWidth', 2.5);
        
        % D. Robot actual y orientacion
        t_act = t(k_anim);
        x_act = x(k_anim);
        y_act = y(k_anim);
        th_act = theta(k_anim);
        
        plot(x_act, y_act, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        largo_flecha = 0.15;
        quiver(x_act, y_act, largo_flecha*cos(th_act), largo_flecha*sin(th_act), ...
            'Color', 'k', 'LineWidth', 2, 'MaxHeadSize', 3);
        
        % E. Titulo dinamico
        str_titulo_1 = sprintf('Animacion Caso %d/8: %s', caso, movimiento);
        str_titulo_2 = sprintf('w_D=%.1f, w_I=%.1f rad/s | u=%.2f m/s, w=%.2f rad/s', ...
            theta_punto_D, theta_punto_I, u, w);
        str_tiempo = sprintf('TIEMPO: %.2f / %.2f s', t_act, ts);
        
        title({str_titulo_1; str_titulo_2; ['\bf' str_tiempo]}, 'FontSize', 12);
        xlabel('X [m]'); ylabel('Y [m]');
        set(gca, 'FontSize', 11);
        
        drawnow; 

        % --- CAPTURA DE FRAMES PARA EL GIF PARA EL CASO ACTUAL ---
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
    
    % =========================================================
    % GUARDAR IMAGEN ESTATICA DE CADA CASO FINALIZADO
    % =========================================================
    if ishandle(figura_animacion)
        nombre_imagen = fullfile(nombre_carpeta, sprintf('Caso_%d_Final.png', caso));
        saveas(figura_animacion, nombre_imagen);
        pause(0.5); 
    else
        break; 
    end
end

%% GRAFICA COMPARATIVA FINAL (ESTATICA)
figura_comparativa = figure('Name', 'Comparacion Final Trayectorias');
set(figura_comparativa, 'Position', [200, 200, 800, 600]);
hold on; grid on; axis equal;
colores = lines(numero_casos); 

for caso = 1:numero_casos
    plot(X_todas(caso,:), Y_todas(caso,:), ...
        'LineWidth', 2, 'Color', colores(caso,:), ...
        'DisplayName', sprintf('Caso %d: %s', caso, tipo_movimiento(caso)));
end
plot(x0, y0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'DisplayName', 'Inicio');
xlabel('Posicion X [m]'); ylabel('Posicion Y [m]');
title('Comparacion de las ocho trayectorias simuladas');
legend('Location', 'bestoutside', 'FontSize', 10);
set(gca, 'FontSize', 12);

nombre_comparativa = fullfile(nombre_carpeta, 'Comparacion_Trayectorias_Final.png');
saveas(figura_comparativa, nombre_comparativa);

%% CREACION DE LA TABLA DE RESULTADOS FINAL (Excel)
fprintf('\nGenerando tabla de resultados y guardando archivos...\n');
Caso = (1:numero_casos)';
w_D_rad_s = velocidades_ruedas(:,1);
w_I_rad_s = velocidades_ruedas(:,2);
u_m_s = u_resultado;
w_rad_s = w_resultado;
Radio_Giro_m = radio_giro;
Movimiento = tipo_movimiento;
X_final_m = x_final;
Y_final_m = y_final;

tabla_resultados = table(Caso, w_D_rad_s, w_I_rad_s, u_m_s, w_rad_s, Radio_Giro_m, X_final_m, Y_final_m, Movimiento);
disp(tabla_resultados);

nombre_archivo_excel = fullfile(nombre_carpeta, 'Resultados_Cinematica_Directa.xlsx');
writetable(tabla_resultados, nombre_archivo_excel);

fprintf('¡Proceso exitoso! Las 8 imagenes, la comparativa, los 8 GIFs y el Excel estan en la carpeta: %s\n', nombre_carpeta);