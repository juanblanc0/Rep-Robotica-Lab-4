%%Juan Camilo Blanco Barrera 
%%Universidad Militar Nueva Granada | 2025-II

clc; clear all;

%%<----------ROBOT ENSAMBLAJE 1---------->%%
syms th1 th2 th3 th4 

a1 = 30;
d1 = 113;

a2 = 200; 
a3 = 140;
a4 = 83;

% Definir DH params en forma [theta, d, a, alpha]
DH = [  th1,        d1,    a1,    pi/2;
        th2+pi/2,    0,    a2,      0;
        th3-pi/2,    0,    a3,      0;
        th4,         0,    a4,      0];

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

T01 = A01;
T02 = A01*A12;
T03 = A01*A12*A23;
T04 = A01*A12*A23*A34;

disp('T01 ='); disp(T01)
disp('T02 ='); disp(T02)
disp('T03 ='); disp(T03)
disp('T04 ='); disp(T04)

%q = [th1; th2; th3; th4];

z0 = [0;0;1];
p0 = [0;0;0];

z1 = [T01(1,3);T01(2,3);T01(3,3)];
p1 = [T01(1,4);T01(2,4);T01(3,4)];

z2 = [T02(1,3);T02(2,3);T02(3,3)];
p2 = [T02(1,4);T02(2,4);T02(3,4)];

z3 = [T03(1,3);T03(2,3);T03(3,3)];
p3 = [T03(1,4);T03(2,4);T03(3,4)];

pe = [T04(1,4);
      T04(2,4);
      T04(3,4)];

J = [cross(z0, (pe-p0)), cross(z1, (pe-p1)), cross(z2, (pe-p2)), cross(z3, (pe-p3));
     z0, z1, z2, z3];

%-----Trayectorias (Descomentar para visualizar la trayectoria especifica)-----%

Ve = 25*[0; 0; -1; 0; 0; 0]; T = 2; div = 50; dt = T/div; qt = [1.5708; 0.2463; -1.0832; -0.7338]; %2
%Ve = 25*[0; 0; 1; 0; 0; 0]; T = 2; div = 50; dt = T/div; qt = [1.5708; -0.7746; -1.5409; 0.7447]; %3
%Ve = 25*[0; 0; -1; 0; 0; 0]; T = 2; div = 50; dt = T/div; qt = [0.6255; -0.0792; -0.8174; -0.6742]; %5
%Ve = 25*[0; 0; 1; 0; 0; 0]; T = 2; div = 50; dt = T/div; qt = [0.6255; -0.4596; -1.0074; -0.1038]; %6
%Ve = 25*[0; 0; -1; 0; 0; 0]; T = 2; div = 50; dt = T/div; qt = [0.2370; -0.1184; -0.7695; -0.6829]; %9
%Ve = 25*[0; 0; 1; 0; 0; 0]; T = 2; div = 50; dt = T/div; qt = [0.2370; -0.4696; -0.9473; -0.1539]; %10
%Ve = 25*[0; 0; -1; 0; 0; 0]; T = 2; div = 50; dt = T/div; qt = [0.4347; -0.2970; -0.5114; -0.7624]; %13
%Ve = 25*[0; 0; 1; 0; 0; 0]; T = 2; div = 50; dt = T/div; qt = [0.4347; -0.5500; -0.6503; -0.3704]; %14

for i=1:div
 
J_eval = double(subs(J, [th1; th2; th3; th4], qt));  % Jacobiano numérico
qp = pinv(J_eval)*Ve;                                % Velocidad
qt1 = qt + qp*dt;                                    % qt1 numérico                    
qt=qt1;

%T = double(subs(T04, [th1; th2; th3; th4], qt));
%coords = [T(1,4), T(2,4), T(3,4)]                    % Coordenadas Efector Final
%or = [T(1,1);T(2,1);T(3,1)]                          % Orientacion Efector Final

fprintf('Iter %03d -> [%.3f, %.3f, %.3f, %.3f]\n', i, qp(1), qp(2), qp(3), qp(4)); % Mostrar velocidades
fprintf('Iter %03d -> [%.3f, %.3f, %.3f, %.3f]\n', i, qt(1), qt(2), qt(3), qt(4)); % Mostrar angulos
end

