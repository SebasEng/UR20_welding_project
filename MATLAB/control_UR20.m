%% 1. Configuración y Solución Numérica
clear; clc; close all;

% Definición de condiciones iniciales (Radianes)
q1i = -pi/2; 
q2i = -pi/2;
q3i = 0; 
q4i = -pi/2;
q5i = 0; 
q6i = pi/2;

qi = [q1i; q2i; q3i; q4i; q5i; q6i]; % Vector de estado inicial

% Tiempo de simulación
h = 1e-3;
ti = 0;
tf = 15; % Tiempo total
tspan = ti:h:tf;

fprintf('Calculando trayectoria con ODE45...\n');

% Resolver la Ecuación Diferencial
% [t, q_rad] contendrá la historia de posiciones (N x 6)
[t, q_rad] = ode45(@ccUR3, tspan, qi);

fprintf('Cálculo finalizado. Iniciando conexión con RoboDK.\n');

%% 2. Conexión con RoboDK
% Inicializamos la API
RDK = Robolink;       
ROBOT_NAME = 'UR20'; % Nombre exacto en tu estación
robot = RDK.Item(ROBOT_NAME);

% Verificación de seguridad
if ~robot.Valid()
    error('El robot "%s" no se encuentra. Revisa RoboDK.', ROBOT_NAME);
end

% Ajustes de velocidad para la simulación (opcional, para suavidad)
%robot.setSpeed(300); % mm/s (lineal) - referencial en simulación
%robot.setSpeedJoints(200); % deg/s

fprintf('Conectado a: %s \n', robot.Name());

%% 3. Animación de Trayectoria en RoboDK
fprintf('Enviando movimientos al robot... \n');

% Iteramos sobre la matriz de resultados 'q_rad'
% Nota: ode45 puede generar muchos puntos, usamos un paso para no saturar
paso_visualizacion = 100; % Graficar cada X puntos para acelerar la vista

for k = 1:paso_visualizacion:length(t)
    disp(k);
    disp(length(t));
    % 1. Extraer configuración actual del vector de tiempos
    q_actual_rad = q_rad(k, :); 
    
    % 2. Convertir a Grados (RoboDK usa grados)
    q_actual_deg = rad2deg(q_actual_rad);
    
    % 3. Enviar comando al robot
    % Usamos MoveJ para mover a la posición calculada
    robot.MoveJ(q_actual_deg);
    
    % Pausa mínima para permitir actualización gráfica si es necesario
    % pause(0.001); 
end

% Mapeamos el estado final real en el simulador
estado_final = robot.Joints();
fprintf('\n*Fin de simulación en RoboDK*\n');
fprintf('Posición final alcanzada (Grados):\n');
disp(estado_final);


%% 4. Función del Sistema (Dinámica de error / Control)
function dq = ccUR3(~, q) % El tiempo 't' no se usa explícitamente en las ecs
    
    % Extracción de estados actuales
    theta1 = q(1); theta2 = q(2); theta3 = q(3); 
    theta4 = q(4); theta5 = q(5); theta6 = q(6);
    
    % --- Posición Actual (Cinemática Directa UR3/UR Series) ---
    % Nota: Asegúrate que estas medidas coincidan con tu UR20 si buscas precisión absoluta
    %x = (201*sin(theta1))/1000 - (431*cos(theta1)*cos(theta2))/500 + (1543*cos(theta5)*sin(theta1))/10000 + (1593*cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/10000 - (1593*sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/10000 + (1543*sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))))/10000 + (7287*cos(theta1)*sin(theta2)*sin(theta3))/10000 - (7287*cos(theta1)*cos(theta2)*cos(theta3))/10000;
    x = (201*sin(theta1))/1000 - (431*cos(theta1)*cos(theta2))/500 + (819752241048395479*cos(theta5)*sin(theta1))/1407374883553280000 + (1593*cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/10000 + (8443997099740547*sin(theta6)*(sin(theta1)*sin(theta5) - cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))))/4611686018427387904 - (1593*sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/10000 + (8443997099740547*cos(theta6)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/4611686018427387904 + (819752241048395479*sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))))/1407374883553280000 + (7287*cos(theta1)*sin(theta2)*sin(theta3))/10000 - (7287*cos(theta1)*cos(theta2)*cos(theta3))/10000;
    y = (1593*cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/10000 - (819752241048395479*cos(theta1)*cos(theta5))/1407374883553280000 - (431*cos(theta2)*sin(theta1))/500 - (201*cos(theta1))/1000 - (1593*sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/10000 - (8443997099740547*sin(theta6)*(cos(theta1)*sin(theta5) + cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))))/4611686018427387904 + (8443997099740547*cos(theta6)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/4611686018427387904 + (819752241048395479*sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))))/1407374883553280000 + (7287*sin(theta1)*sin(theta2)*sin(theta3))/10000 - (7287*cos(theta2)*cos(theta3)*sin(theta1))/10000;
    z = (1593*sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/10000 - (7287*cos(theta2)*sin(theta3))/10000 - (7287*cos(theta3)*sin(theta2))/10000 - (8443997099740547*cos(theta6)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/4611686018427387904 - (819752241048395479*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))))/1407374883553280000 - (1593*cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/10000 - (431*sin(theta2))/500 + (8443997099740547*cos(theta5)*sin(theta6)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))))/4611686018427387904 + 2363/10000;
    %y = (1593*cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/10000 - (1543*cos(theta1)*cos(theta5))/10000 - (431*cos(theta2)*sin(theta1))/500 - (201*cos(theta1))/1000 - (1593*sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/10000 + (1543*sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))))/10000 + (7287*sin(theta1)*sin(theta2)*sin(theta3))/10000 - (7287*cos(theta2)*cos(theta3)*sin(theta1))/10000;
    %z = (1593*sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/10000 - (7287*cos(theta2)*sin(theta3))/10000 - (7287*cos(theta3)*sin(theta2))/10000 - (1543*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))))/10000 - (1593*cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/10000 - (431*sin(theta2))/500 + 2363/10000;
    
    X = [x; y; z];
    
    % --- Variables Deseadas (Setpoint Fijo) ---
    xd = 1.0; 
    yd = 0.125;
    zd = 1.252889;
    Xd = [xd; yd; zd];
    dXd = [0; 0; 0]; 
    
    % --- Cálculo del Jacobiano (J) ---
    j11 = (201*cos(theta1))/1000 + (819752241048395479*cos(theta1)*cos(theta5))/1407374883553280000 + (431*cos(theta2)*sin(theta1))/500 - (1593*cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/10000 + (1593*sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/10000 + (8443997099740547*sin(theta6)*(cos(theta1)*sin(theta5) + cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))))/4611686018427387904 - (8443997099740547*cos(theta6)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/4611686018427387904 - (819752241048395479*sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))))/1407374883553280000 - (7287*sin(theta1)*sin(theta2)*sin(theta3))/10000 + (7287*cos(theta2)*cos(theta3)*sin(theta1))/10000;
    j12 = (431*cos(theta1)*sin(theta2))/500 - (1593*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/10000 - (1593*sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/10000 - (8443997099740547*cos(theta6)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))))/4611686018427387904 + (819752241048395479*sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/1407374883553280000 - (8443997099740547*cos(theta5)*sin(theta6)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/4611686018427387904 + (7287*cos(theta1)*cos(theta2)*sin(theta3))/10000 + (7287*cos(theta1)*cos(theta3)*sin(theta2))/10000;
    j13 = (819752241048395479*sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/1407374883553280000 - (1593*sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/10000 - (8443997099740547*cos(theta6)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))))/4611686018427387904 - (1593*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/10000 - (8443997099740547*cos(theta5)*sin(theta6)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/4611686018427387904 + (7287*cos(theta1)*cos(theta2)*sin(theta3))/10000 + (7287*cos(theta1)*cos(theta3)*sin(theta2))/10000;
    j14 = (819752241048395479*sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/1407374883553280000 - (1593*sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/10000 - (8443997099740547*cos(theta6)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))))/4611686018427387904 - (1593*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/10000 - (8443997099740547*cos(theta5)*sin(theta6)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/4611686018427387904;
    j15 = (8443997099740547*sin(theta6)*(cos(theta5)*sin(theta1) + sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))))/4611686018427387904 - (819752241048395479*sin(theta1)*sin(theta5))/1407374883553280000 + (819752241048395479*cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))))/1407374883553280000;
    j16 = (8443997099740547*cos(theta6)*(sin(theta1)*sin(theta5) - cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))))/4611686018427387904 - (8443997099740547*sin(theta6)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/4611686018427387904;
    
    j21 = (201*sin(theta1))/1000 - (431*cos(theta1)*cos(theta2))/500 + (819752241048395479*cos(theta5)*sin(theta1))/1407374883553280000 + (1593*cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/10000 + (8443997099740547*sin(theta6)*(sin(theta1)*sin(theta5) - cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))))/4611686018427387904 - (1593*sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/10000 + (8443997099740547*cos(theta6)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/4611686018427387904 + (819752241048395479*sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))))/1407374883553280000 + (7287*cos(theta1)*sin(theta2)*sin(theta3))/10000 - (7287*cos(theta1)*cos(theta2)*cos(theta3))/10000;
    j22 = (431*sin(theta1)*sin(theta2))/500 - (1593*cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/10000 - (1593*sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/10000 - (8443997099740547*cos(theta6)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))))/4611686018427387904 + (819752241048395479*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/1407374883553280000 + (7287*cos(theta2)*sin(theta1)*sin(theta3))/10000 + (7287*cos(theta3)*sin(theta1)*sin(theta2))/10000 - (8443997099740547*cos(theta5)*sin(theta6)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/4611686018427387904;
    j23 = (819752241048395479*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/1407374883553280000 - (1593*sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/10000 - (8443997099740547*cos(theta6)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))))/4611686018427387904 - (1593*cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/10000 + (7287*cos(theta2)*sin(theta1)*sin(theta3))/10000 + (7287*cos(theta3)*sin(theta1)*sin(theta2))/10000 - (8443997099740547*cos(theta5)*sin(theta6)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/4611686018427387904;
    j24 = (819752241048395479*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/1407374883553280000 - (1593*sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/10000 - (8443997099740547*cos(theta6)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))))/4611686018427387904 - (1593*cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/10000 - (8443997099740547*cos(theta5)*sin(theta6)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/4611686018427387904;
    j25 = (819752241048395479*cos(theta1)*sin(theta5))/1407374883553280000 - (8443997099740547*sin(theta6)*(cos(theta1)*cos(theta5) - sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))))/4611686018427387904 + (819752241048395479*cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))))/1407374883553280000;
    j26 = - (8443997099740547*cos(theta6)*(cos(theta1)*sin(theta5) + cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))))/4611686018427387904 - (8443997099740547*sin(theta6)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/4611686018427387904;
    
    j31 = 0;
    j32 = (7287*sin(theta2)*sin(theta3))/10000 - (7287*cos(theta2)*cos(theta3))/10000 - (431*cos(theta2))/500 + (8443997099740547*cos(theta6)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))))/4611686018427387904 - (819752241048395479*sin(theta5)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/1407374883553280000 + (1593*cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/10000 + (1593*sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/10000 + (8443997099740547*cos(theta5)*sin(theta6)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/4611686018427387904;
    j33 = (7287*sin(theta2)*sin(theta3))/10000 - (7287*cos(theta2)*cos(theta3))/10000 + (8443997099740547*cos(theta6)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))))/4611686018427387904 - (819752241048395479*sin(theta5)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/1407374883553280000 + (1593*cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/10000 + (1593*sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/10000 + (8443997099740547*cos(theta5)*sin(theta6)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/4611686018427387904;
    j34 = (8443997099740547*cos(theta6)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))))/4611686018427387904 - (819752241048395479*sin(theta5)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/1407374883553280000 + (1593*cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/10000 + (1593*sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/10000 + (8443997099740547*cos(theta5)*sin(theta6)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/4611686018427387904;
    j35 = - (819752241048395479*cos(theta5)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))))/1407374883553280000 - (8443997099740547*sin(theta5)*sin(theta6)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))))/4611686018427387904;
    j36 = (8443997099740547*sin(theta6)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/4611686018427387904 + (8443997099740547*cos(theta5)*cos(theta6)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))))/4611686018427387904;
    
    J = [j11 j12 j13 j14 j15 j16
         j21 j22 j23 j24 j25 j26
         j31 j32 j33 j34 j35 j36];
         
    Ji = pinv(J);
    
    % --- Ganancia de Control (K) ---
    K = [1 0 0; 0 1 0; 0 0 1]; 
    
    dq = Ji*(dXd - K*(X-Xd));
end
