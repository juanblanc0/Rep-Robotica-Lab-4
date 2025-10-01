clc; clear; syms th1 th2 th3 th4 

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

T04 = A01*A12*A23*A34;
T03 = A01*A12*A23;
T02 = A01*A12;
T01 = A01;
disp('T01 ='); disp(T01)
disp('T02 ='); disp(T02)
disp('T03 ='); disp(T03)
disp('T04 ='); disp(T04)

q = [th1; th2; th3; th4];

pe = [T04(1,4);
      T04(2,4);
      T04(3,4)];

J = jacobian(pe,q);

Ve=25*[0; 0;-1];
T = 2;
div = 50;
dt=T/div;

qt = [1.5708 ;   0.2463;  -1.0832;   -0.7338]
%    1.5708   -0.7746   -1.5409    0.7447

for i=1:div
disp('i ='); disp(i)   
J_eval = double(subs(J, [th1; th2; th3; th4], qt));  % Jacobiano numérico
qp = pinv(J_eval)*Ve; %Velocidad
qt1 = qt + qp*dt; % qt1 numérico                    
qt=qt1;
T = double(subs(T04, [th1; th2; th3; th4], qt));
%coords = [T(1,4), T(2,4), T(3,4)];
or = [T(1,1);T(2,1);T(3,1)]


end