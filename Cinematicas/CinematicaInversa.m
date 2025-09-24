clear all; clc;

%%ROBOT ENSAMBLAJE 1

%Recoleccion = (0,70,70+83)
%Punto 1 = (63+45, 78, 70+83)
%Punto 2 = (63+75, 33.33, 70+83)
%Punto 3 = (63+105, 78, 70+83)

coordX = 63+105;
coordY = 78;
coordZ = 70+83;

a1 = 30;
d1 = 113;

a2 = 200; 
a3 = 140;
a4 = 83;


theta1 = atan2(coordY, coordX)

Pwx1 = sqrt((coordX)^2+(coordY)^2)-sqrt((a1*cos(theta1))^2+(a1*sin(theta1))^2);

r = sqrt((Pwx1)^2+(coordZ-d1)^2);

alpha = atan2((coordZ-d1), Pwx1);

beta = acos(((a2^2)+(r^2)-(a3^2))/(2*a2*r))

gamma = acosd(((a2^2)+(a3^2)-(r^2))/(2*a2*a3));

theta2P = alpha+beta-(pi/2)
theta2M = alpha-beta-(pi/2)

theta3P = deg2rad((180-gamma))+(pi/2)
theta3M = deg2rad(-(180-gamma))+(pi/2)

theta4 = -((theta2P+theta3M)+(pi/2))

GradRes = [theta1 theta2P theta3M]
GradResFull = [theta1 theta2P theta3M theta4]

GradURDF = rad2deg(GradResFull)

%%----------COMPROBACION----------%%
syms th1 th2 th3 th4

% Definir DH params en forma [theta, d, a, alpha]
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

T03 = A01*A12*A23;
T04 = A01*A12*A23*A34;

% Sustituir en T03
T_home = subs(T03, [th1 th2 th3], GradRes);

disp('T03 en punto 1 = ');
disp(vpa(T_home, 4))   % vpa para ver con 4 decimales

% Sustituir en T04
T_home = subs(T04, [th1 th2 th3 th4], GradResFull);

disp('T04 en punto 1 = ');
disp(vpa(T_home, 4))   % vpa para ver con 4 decimales
