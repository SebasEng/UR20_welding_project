
clear                
clc                   
close all             

%% --- 1. CONEXIÓN Y MOVIMIENTO CON ROBODK ---

fprintf('--- 1. Conectando a RoboDK y moviendo el robot ---\n');
RDK = Robolink;       % Hacemos la conexión Matlab-RoboDK


ROBOT_NAME = 'UR20';
ur20_proyecto_final = RDK.Item(ROBOT_NAME); % Creamos el objeto robot en MATLAB


if ~ur20_proyecto_final.Valid()
    error('El robot "%s" no pudo ser encontrado. Verifique que RoboDK esté abierto.', ROBOT_NAME);
end

if ur20_proyecto_final.Type() ~= RDK.ITEM_TYPE_ROBOT
    error('El item "%s" existe, pero NO es un Robot (Tipo: %d). Revise la jerarquía.', ROBOT_NAME, ur20_proyecto_final.Type());
end

fprintf('Robot configurado:\t %s \n', ur20_proyecto_final.Name());



estado_final_grados = ur20_proyecto_final.Joints();

fprintf('\n*Fin de simulación RoboDK*\n');
fprintf('Estado Final (Grados):\t [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', estado_final_grados);


%% --- 6. CÁLCULO DE CINEMÁTICA DIRECTA (DH) ---

fprintf('\n--- 2. Calculando Cinemática Directa con los ángulos de RoboDK ---\n');

% Parámetros DH del robot UR3
% Tralacionales/longitudes en metros
a1 = 0; a2 = -0.8620; a3 = -0.7287; a4=0; a5=0; a6= 0;
d1 = 0.2363; d2 = 0; d3= 0; d4 = 0.2010; d5=0.1593; d6=0.1543;
% Rotacionales (en radianes)
alpha1 = pi/2; alpha2 = 0; alpha3 = 0; alpha4 = pi/2; alpha5 = -pi/2; alpha6 = 0;

% ⚠️ ESTA ES LA PARTE CLAVE DE LA UNIÓN ⚠️
% Reemplazamos los thetas fijos con los valores leídos de RoboDK
% Los convertimos de grados (RoboDK) a radianes (Cálculo DH)
theta1 = deg2rad(estado_final_grados(1));
theta2 = deg2rad(estado_final_grados(2));
theta3 = deg2rad(estado_final_grados(3));
theta4 = deg2rad(estado_final_grados(4));
theta5 = deg2rad(estado_final_grados(5));
theta6 = deg2rad(estado_final_grados(6));

fprintf('Usando ángulos en Radianes: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', ...
        theta1, theta2, theta3, theta4, theta5, theta6);

% --- 7. CÁLCULO DE MATRICES DE TRANSFORMACIÓN ---

%Transformacion para base... i=1
T01 = [cos(theta1) -sin(theta1)*cos(alpha1) sin(theta1)*sin(alpha1) a1*cos(theta1)
    sin(theta1) cos(theta1)*cos(alpha1) -cos(theta1)*sin(alpha1) a1*sin(theta1)
    0 sin(alpha1) cos(alpha1) d1
    0 0 0 1];

T12 = [cos(theta2) -sin(theta2)*cos(alpha2) sin(theta2)*sin(alpha2) a2*cos(theta2)
    sin(theta2) cos(theta2)*cos(alpha2) -cos(theta2)*sin(alpha2) a2*sin(theta2)
    0 sin(alpha2) cos(alpha2) d2
    0 0 0 1];

T23 = [cos(theta3) -sin(theta3)*cos(alpha3) sin(theta3)*sin(alpha3) a3*cos(theta3)
    sin(theta3) cos(theta3)*cos(alpha3) -cos(theta3)*sin(alpha3) a3*sin(theta3)
    0 sin(alpha3) cos(alpha3) d3
    0 0 0 1];

T34 = [cos(theta4) -sin(theta4)*cos(alpha4) sin(theta4)*sin(alpha4) a4*cos(theta4)
    sin(theta4) cos(theta4)*cos(alpha4) -cos(theta4)*sin(alpha4) a4*sin(theta4)
    0 sin(alpha4) cos(alpha4) d4
    0 0 0 1];

T45 = [cos(theta5) -sin(theta5)*cos(alpha5) sin(theta5)*sin(alpha5) a5*cos(theta5)
    sin(theta5) cos(theta5)*cos(alpha5) -cos(theta5)*sin(alpha5) a5*sin(theta5)
    0 sin(alpha5) cos(alpha5) d5
    0 0 0 1];

T56 = [cos(theta6) -sin(theta6)*cos(alpha6) sin(theta6)*sin(alpha6) a6*cos(theta6)
    sin(theta6) cos(theta6)*cos(alpha6) -cos(theta6)*sin(alpha6) a6*sin(theta6)
    0 sin(alpha6) cos(alpha6) d6
    0 0 0 1];

%con efector final
%Trnsformacion a efector final o tcp

T67 = [ 1 0 0 0;
        0 1 0 0;
        0 0 1 .428169;
        0 0 0 1];

T78 = [ 1 0 0 0;
        0 1 0 -0.001831;
        0 0 1 0;
        0 0 0 1];

T_F = [ cos(pi/4) 0 sin(pi/4) 0;
        0 1 0 0;
        -sin(pi/4) 0 cos(pi/4) 0;
        0 0 0 1];

TEF = T67*T78*T_F;

% Matriz de Cinemática Directa Final
CD = T01 * T12 * T23 * T34 * T45 * T56* TEF;

%% --- 8. RESULTADOS ---
fprintf('\n--- 3. Resultados Finales ---\ n');
fprintf('Matriz de Transformación Homogénea Final (CD) calculada:\n');
disp(CD);


fprintf('\n*Fin de script combinado*\n');
