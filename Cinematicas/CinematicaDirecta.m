clc; clear; syms th1 th2 th3 th4 

% Definir DH params en forma [theta, d, a, alpha]
DH = [  th1,        113,   -30,    pi/2;
        th2+pi/2,     0, 200,      0;
        th3-pi/2,           0, 140,   0;
        th4,         0,   110,  0];

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

% Mostrar resultados
disp('A01 ='); disp(A01)
disp('A12 ='); disp(A12)
disp('A23 ='); disp(A23)
disp('A34 ='); disp(A34)

T04 = A01*A12*A23*A34;
T03 = A01*A12*A23;
disp('T04 ='); disp(T04)
disp('T03 ='); disp(T03)

% Definir el punto home
q_1 = [1.5708    0.1861    2.8604];

% Sustituir en T04
T_home = subs(T03, [th1 th2 th3], q_1);

disp('T03 en punto 1 = ');
disp(vpa(T_home, 4))   % vpa para ver con 4 decimales

q_2 = [-pi/2, pi/9, -pi/6, -(4*pi/9)];

% Sustituir en T04
T_1 = subs(T04, [th1 th2 th3 th4], q_2);

disp('T04 en punto 2 = ');
disp(vpa(T_1, 4))   % vpa para ver con 4 decimales
