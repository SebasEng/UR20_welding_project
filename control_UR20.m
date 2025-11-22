%% 1. Configuración y Simulación
clear; clc; close all;

% Definición de condiciones iniciales
q1i = 0; 
q2i = -pi/2;
q3i = -pi/2; % Vector columna
q4i = 0;
q5i = pi/2;
q6i = 0;

qi = [q1i; q2i; q3i; q4i; q5i; q6i];

% Tiempo de simulación
h = 1e-3;
ti = 0;
tf = 30;
tspan = ti:h:tf;

% Resolver la Ecuación Diferencial (ODE45)
% La función 'ccRR' DEBE devolver el vector de velocidades articulares dq = [dq1; dq2]
[t, q] = ode45(@ccUR3, tspan, qi);

qd = rad2deg(q)

%% 4. Función del Sistema (Dinámica de error / Control)
function dq = ccUR3(t,q)
    
    % Extracción de estados actuales
    theta1 = q(1); 
    theta2 = q(2);
    theta3 = q(3); 
    theta4 = q(4);
    theta5 = q(5);
    theta6 = q(6);
    %simple uristica, cruvas parametricas, polinomio quintico
    
    % --- Posición Actual (Cinemática Directa) ---
    x = (201*sin(theta1))/1000 - (431*cos(theta1)*cos(theta2))/500 + (1543*cos(theta5)*sin(theta1))/10000 + (1593*cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/10000 - (1593*sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/10000 + (1543*sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))))/10000 + (7287*cos(theta1)*sin(theta2)*sin(theta3))/10000 - (7287*cos(theta1)*cos(theta2)*cos(theta3))/10000;
    y = (1593*cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/10000 - (1543*cos(theta1)*cos(theta5))/10000 - (431*cos(theta2)*sin(theta1))/500 - (201*cos(theta1))/1000 - (1593*sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/10000 + (1543*sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))))/10000 + (7287*sin(theta1)*sin(theta2)*sin(theta3))/10000 - (7287*cos(theta2)*cos(theta3)*sin(theta1))/10000;
    z = (1593*sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/10000 - (7287*cos(theta2)*sin(theta3))/10000 - (7287*cos(theta3)*sin(theta2))/10000 - (1543*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))))/10000 - (1593*cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/10000 - (431*sin(theta2))/500 + 2363/10000;
    X = [x; y; z];
    
    % --- Variables Deseadas (Setpoint Fijo) ---
    xd = -0.301337; 
    yd = -0.385295;
    zd = 1.114727;
    Xd = [xd; yd; zd];
    dXd = [0; 0; 0]; % Velocidad Deseada (0)
    
    % --- Cálculo del Jacobiano (J) ---
    

    j11 = (201*cos(theta1))/1000 + (1543*cos(theta1)*cos(theta5))/10000 + (431*cos(theta2)*sin(theta1))/500 - (1593*cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/10000 + (1593*sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/10000 - (1543*sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))))/10000 - (7287*sin(theta1)*sin(theta2)*sin(theta3))/10000 + (7287*cos(theta2)*cos(theta3)*sin(theta1))/10000;
    j12 = (431*cos(theta1)*sin(theta2))/500 - (1593*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/10000 - (1593*sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/10000 + (1543*sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/10000 + (7287*cos(theta1)*cos(theta2)*sin(theta3))/10000 + (7287*cos(theta1)*cos(theta3)*sin(theta2))/10000;
    j13 = (1543*sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/10000 - (1593*sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/10000 - (1593*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/10000 + (7287*cos(theta1)*cos(theta2)*sin(theta3))/10000 + (7287*cos(theta1)*cos(theta3)*sin(theta2))/10000;
    j14 = (1543*sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))))/10000 - (1593*sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/10000 - (1593*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/10000;
    j15 = (1543*cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))))/10000 - (1543*sin(theta1)*sin(theta5))/10000;
    j16 = 0;
    
    j21 = (201*sin(theta1))/1000 - (431*cos(theta1)*cos(theta2))/500 + (1543*cos(theta5)*sin(theta1))/10000 + (1593*cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/10000 - (1593*sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/10000 + (1543*sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))))/10000 + (7287*cos(theta1)*sin(theta2)*sin(theta3))/10000 - (7287*cos(theta1)*cos(theta2)*cos(theta3))/10000;
    j22 = (431*sin(theta1)*sin(theta2))/500 - (1593*cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/10000 - (1593*sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/10000 + (1543*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/10000 + (7287*cos(theta2)*sin(theta1)*sin(theta3))/10000 + (7287*cos(theta3)*sin(theta1)*sin(theta2))/10000;
    j23 = (1543*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/10000 - (1593*sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/10000 - (1593*cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/10000 + (7287*cos(theta2)*sin(theta1)*sin(theta3))/10000 + (7287*cos(theta3)*sin(theta1)*sin(theta2))/10000;
    j24 = (1543*sin(theta5)*(cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))))/10000 - (1593*sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/10000 - (1593*cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/10000;
    j25 = (1543*cos(theta1)*sin(theta5))/10000 + (1543*cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))))/10000;
    j26 = 0;
    
    j31 = 0;
    j32 = (7287*sin(theta2)*sin(theta3))/10000 - (7287*cos(theta2)*cos(theta3))/10000 - (431*cos(theta2))/500 - (1543*sin(theta5)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/10000 + (1593*cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/10000 + (1593*sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/10000;
    j33 = (7287*sin(theta2)*sin(theta3))/10000 - (7287*cos(theta2)*cos(theta3))/10000 - (1543*sin(theta5)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/10000 + (1593*cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/10000 + (1593*sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/10000;
    j34 = (1593*cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/10000 - (1543*sin(theta5)*(cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))))/10000 + (1593*sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/10000;
    j35 = -(1543*cos(theta5)*(cos(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))))/10000;
    j36 = 0;


    J = [j11 j12 j13 j14 j15 j16
         j21 j22 j23 j24 j25 j26
         j31 j32 j33 j34 j35 j36];

    Ji = pinv(J);
    
    % --- Ganancia de Control (K) ---
    K = [1 0 0; 0 1 0; 0 0 1]; 
    
    dq = Ji*(dXd - K*(X-Xd));

end % Fin de la función ccRR