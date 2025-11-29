clc; clear; close all;
%% 1. CONEXIÓN CON ROBODK
fprintf('Iniciando conexión con RoboDK...\n');
% Inicializar API
RDK = Robolink;       
ROBOT_NAME = 'UR20'; % Asegúrate que este sea el nombre en tu árbol de RoboDK
% Buscar el robot
robot = RDK.Item(ROBOT_NAME);
% Verificaciones de seguridad
if ~robot.Valid()
    error('El robot "%s" no se encuentra en la estación. Revisa el nombre.', ROBOT_NAME);
end
fprintf('Conectado exitosamente a: %s \n', robot.Name());
% Opcional: Ajustar velocidad de la simulación para ver el movimiento claro
robot.setSpeed(100); % mm/s (velocidad lineal del TCP)
robot.setSpeedJoints(50); % deg/s (velocidad articular)
%% 2. DEFINICIÓN DEL PUNTO OBJETIVO (INPUT USUARIO)
% Escribe aquí las coordenadas a las que quieres ir (en METROS)
target_x = -0.6; 
target_y = -0.7;
target_z = 0.9; 
fprintf('------------------------------------------------\n');
fprintf('Calculando Cinemática Inversa para el punto:\n');
fprintf('X: %.4f | Y: %.4f | Z: %.4f\n', target_x, target_y, target_z);
fprintf('------------------------------------------------\n');
%% 3. CÁLCULO DE CINEMÁTICA INVERSA (Modelo Geométrico)
% --- Parámetros del Robot (UR20 según tu código) ---
d1=0.2363; 
d2=0; 
d3=0; 
d4=0.2010;
d5=0.1593; 
% NOTA: Aquí sumaste un offset de herramienta en tu código original. 
% Mantengo tu lógica: d6_robot + longitud_herramienta
d6 = 0.1543 + 0.428169; 
% Longitudes de eslabones (UR20)
a2 = -0.8620; 
a3 = -0.7287;
% --- Configuración de Orientación (Fija: Ataque Vertical hacia abajo) ---
% Matriz de rotación deseada para atacar hacia abajo (eje Z apuntando a -Z global)
nx = 1;  ox = 0;  ax = 0;
ny = 0;  oy = 1;  ay = 0;
nz = 0;  oz = 0;  az = -1; 
px = target_x;
py = target_y;
pz = target_z;
% --- ALGORITMO DE INVERSA (Tus ecuaciones) ---
% 1. Cálculo de q1 (Base)
% Se elige una de las soluciones posibles (codo arriba/abajo, izquierda/derecha)
q1 = atan2(py - ay*d6, px - ax*d6) - atan2(-d4, sqrt((px - ax*d6)^2 + (py - ay*d6)^2 - (-d4)^2));
% 2. Operaciones auxiliares para la Muñeca (q5, q6)
S5 = sqrt((-sin(q1)*nx + cos(q1)*ny)^2 + (-sin(q1)*ox + cos(q1)*oy)^2);
% 3. Cálculo de q5
q5 = atan2(S5, sin(q1)*ax - cos(q1)*ay);
% 4. Cálculo de q6
num_q6_y = (-sin(q1)*ox + cos(q1)*oy) / S5;
num_q6_x = ( sin(q1)*nx - cos(q1)*ny) / S5;
q6 = atan2(num_q6_y, num_q6_x);
% 5. Operaciones auxiliares para el Brazo (q2, q3, q4)
q234 = atan2((-az/S5), (-(cos(q1)*ax + sin(q1)*ay)/S5));
B1 = (cos(q1)*px + sin(q1)*py - d5*sin(q234) + d6*cos(q234)) * S5;
B2 = (pz - d1 + d5*cos(q234) + d6*sin(q234)) * S5;
A_val = -2 * (pz - d1 + d5*cos(q234) + d6*sin(q234)) * S5 * a2;
B_val =  2 * (cos(q1)*px + sin(q1)*py - d5*sin(q234) + d6*cos(q234)) * S5 * a2;
C_val = B1^2 + B2^2 + a2^2 - a3^2;
% Validación de alcanzabilidad (si el punto está fuera del alcance)
if (A_val^2 + B_val^2 - C_val^2) < 0
    error('El punto deseado está FUERA del alcance del robot (Singularidad matemática).');
end
% 6. Cálculo de q2 (Hombro)
q2 = atan2(B_val, A_val) - atan2(C_val, sqrt(A_val^2 + B_val^2 - C_val^2));
% 7. Cálculo de q3 (Codo)
q23 = atan2((B2 - a2*sin(q2))/a3, (B1 - a2*cos(q2))/a3);
q3 = q23 - q2;
% 8. Cálculo de q4 (Muñeca 1)
q4 = q234 - q23;
%% 4. ENVIAR COMANDO A ROBODK
% Convertir a grados (RoboDK trabaja en grados)
q_deg = rad2deg([q1, q2, q3, q4, q5, q6]);
fprintf('\nÁngulos calculados (Grados):\n');
disp(q_deg);
fprintf('Moviendo robot...\n');
% Comando de movimiento articular (Joint Move)
% Esto mueve el robot directamente a la configuración calculada
robot.MoveJ(q_deg);
fprintf('¡Movimiento completado!\n');
