clc; clear; close all;

%% 1. DEFINICIÓN DEL PUNTO OBJETIVO /yo lo pongo
% Cambia estos valores por la posición a la que quieres ir:
target_x = -0.30125; %0.599514;  
target_y = -0.384936;%-1.094933;
target_z = 1.11635;%0.456613; 

fprintf('------------------------------------------------\n');
fprintf('Calculando Cinemática Inversa para el punto:\n');
fprintf('X: %.4f | Y: %.4f | Z: %.4f\n', target_x, target_y, target_z);
fprintf('------------------------------------------------\n');

%% 2. Parámetros del Robot (DH y Geometría)
d1=0.2363; d2=0; d3=0; d4=0.2010;
d5=0.1593; d6=0.1543; d7=0;

% Longitudes de eslabones
a0=0; a1=0; a2=-0.8620; a3=-0.7287;
a4=0; a5=0; a6=0;

%% 3. Configuración de Orientación (Fija: Apuntando Abajo)
% Para resolver IK necesitamos posición y orientación. 
% Usamos la misma del código original (Matriz de Rotación fija).
nx = 1;  ox = 0;  ax = 0;
ny = 0;  oy = 1;  ay = 0;
nz = 0;  oz = 0;  az = -1; % Eje de ataque hacia abajo (Z negativo)

% Asignamos variables para facilitar lectura de fórmulas
px = target_x;
py = target_y;
pz = target_z;

%% 4. Cálculos de Cinemática Inversa

% --- Cálculo de q1 (Base) ---
q1 = atan2(py - ay*d6, px - ax*d6) - atan2(-d4, sqrt((px - ax*d6)^2 + (py - ay*d6)^2 - (-d4)^2));

% --- Operaciones auxiliares para la Muñeca (q5, q6) ---
S5 = sqrt((-sin(q1)*nx + cos(q1)*ny)^2 + (-sin(q1)*ox + cos(q1)*oy)^2);

% --- Cálculo de q5 ---
q5 = atan2(S5, sin(q1)*ax - cos(q1)*ay);

% --- Cálculo de q6 ---
num_q6_y = (-sin(q1)*ox + cos(q1)*oy) / S5;
num_q6_x = ( sin(q1)*nx - cos(q1)*ny) / S5;
q6 = atan2(num_q6_y, num_q6_x);

% --- Operaciones auxiliares para el Brazo (q2, q3, q4) ---
% Ángulo global de la muñeca
q234 = atan2((-az/S5), (-(cos(q1)*ax + sin(q1)*ay)/S5));

% Variables para la Ley de Cosenos (Geometría del brazo)
B1 = (cos(q1)*px + sin(q1)*py - d5*sin(q234) + d6*cos(q234)) * S5;
B2 = (pz - d1 + d5*cos(q234) + d6*sin(q234)) * S5;

A_val = -2 * (pz - d1 + d5*cos(q234) + d6*sin(q234)) * S5 * a2;
B_val =  2 * (cos(q1)*px + sin(q1)*py - d5*sin(q234) + d6*cos(q234)) * S5 * a2;
C_val = B1^2 + B2^2 + a2^2 - a3^2;

% --- Cálculo de q2 (Hombro) ---
q2 = atan2(B_val, A_val) - atan2(C_val, sqrt(A_val^2 + B_val^2 - C_val^2));

% --- Cálculo de q3 (Codo) ---
q23 = atan2((B2 - a2*sin(q2))/a3, (B1 - a2*cos(q2))/a3);
q3 = q23 - q2;

% --- Cálculo de q4 (Muñeca arriba/abajo) ---
q4 = q234 - q23;

%% 5. Resultado Final (Conversión a Grados)
deg = 180/pi; % Factor de conversión

q1_deg = q1 * deg;
q2_deg = q2 * deg;
q3_deg = q3 * deg;
q4_deg = q4 * deg;
q5_deg = q5 * deg;
q6_deg = q6 * deg;

fprintf('\nRESULTADOS (Ángulos articulares requeridos):\n');
fprintf('q1 (Base):    %8.4f grados\n', q1_deg);
fprintf('q2 (Hombro):  %8.4f grados\n', q2_deg);
fprintf('q3 (Codo):    %8.4f grados\n', q3_deg);
fprintf('q4 (Muñeca1): %8.4f grados\n', q4_deg);
fprintf('q5 (Muñeca2): %8.4f grados\n', q5_deg);
fprintf('q6 (Muñeca3): %8.4f grados\n', q6_deg);
fprintf('------------------------------------------------\n');