%%Juan Camilo Blanco Barrera 
%%Universidad Militar Nueva Granada | 2025-II

clear all; clc;

%%<----------ROBOT ENSAMBLAJE 1---------->%%
a1 = 30;
d1 = 113;

a2 = 200; 
a3 = 140;
a4 = 83;

Ac = 50;
%%-----------Set Coordenadas-----------%%
Home = [140+30, 0, 113+200];

Rec = [0, 75, 70+a4];
RecAc = [0, 75, 70+a4+Ac];

P1 = [63+45, 78, 70+a4];
P1Ac = [63+45, 78, 70+a4+Ac];

P2 = [63+75, 33.333, 70+a4];
P2Ac = [63+75, 33.333, 70+a4+Ac];

P3 = [63+105, 78, 70+a4];
P3Ac = [63+105, 78, 70+a4+Ac];
%%-------------------------------------%%

coords = RecAc;

coordX = coords(1,1); coordY = coords(1,2); coordZ = coords(1,3);

theta1 = atan2(coordY, coordX);

Pwx1 = sqrt((coordX)^2+(coordY)^2)-sqrt((a1*cos(theta1))^2+(a1*sin(theta1))^2);

r = sqrt((Pwx1)^2+(coordZ-d1)^2);

alpha = atan2((coordZ-d1), Pwx1);

beta = acos(((a2^2)+(r^2)-(a3^2))/(2*a2*r));

gamma = acosd(((a2^2)+(a3^2)-(r^2))/(2*a2*a3));

theta2 = alpha+beta-(pi/2);             % theta2 para codo arriba

theta3 = deg2rad(-(180-gamma))+(pi/2);  % theta3 para codo arriba

theta4 = -((theta2+theta3)+(pi/2));

GradRad = [theta1 theta2 theta3 theta4] % Angulos para todos los GDL (Radianes)
GradDeg = rad2deg(GradRad);             % Angulos para todos los GDL (Grados)

%%---------------Comprobacion--------------%%
syms th1 th2 th3 th4

%Matriz DH [theta, d, a, alpha]
DH = [  th1,        d1,   a1,    pi/2;
        th2+pi/2,    0,    a2,      0;
        th3-pi/2,    0,    a3,      0;
        th4,         0,   a4,      0];

% Función para matriz homogénea
A = @(theta,d,a,alpha)[cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
                       sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                       0            sin(alpha)             cos(alpha)            d;
                       0            0                      0                     1];

% Calcular matrices
A01 = simplify(A(DH(1,1), DH(1,2), DH(1,3), DH(1,4)));
A12 = simplify(A(DH(2,1), DH(2,2), DH(2,3), DH(2,4)));
A23 = simplify(A(DH(3,1), DH(3,2), DH(3,3), DH(3,4)));
A34 = simplify(A(DH(4,1), DH(4,2), DH(4,3), DH(4,4)));

T04 = A01*A12*A23*A34;

% Sustituir en T04
T_home = subs(T04, [th1 th2 th3 th4], GradRad);

disp('T04 en Coordenadas = ');
disp(vpa(T_home, 4))   % vpa para ver con 4 decimales
