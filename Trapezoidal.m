%%Juan Camilo Blanco Barrera 
%%Universidad Militar Nueva Granada | 2025-II

clc; clear all;

%%<----------ROBOT ENSAMBLAJE 1---------->%%

%%--------------Sets Angulos--------------%%

Home = [0, 0, 0, -1.5708];

Rec = [1.5708, -0.7746, -1.5409, 0.7447];
RecAc = [1.5708, 0.2463, -1.0832, -0.7338];

P1 = [0.6255, -0.4596, -1.0074, -0.1038];
P1Ac = [0.6255, -0.0792, -0.8174, -0.6742];

P2 = [0.2370, -0.4696, -0.9473, -0.1539];
P2Ac = [0.2370, -0.1184, -0.7695, -0.6829];

P3 = [0.4347, -0.5500, -0.6503, -0.3704];
P3Ac = [0.4347, -0.2970, -0.5114, -0.7624];

tf = 1; % Tiempo final

qc_p = zeros(4, 1);
tc = zeros(4,1);
qc_2p = zeros(4, 1);

%%-------------Trayectorias-------------%%

T1 = [Home; RecAc];
T4 = [RecAc; P1Ac];
T7 = [P1Ac; Home];
T8 = [RecAc; P2Ac];
T11 = [P2Ac; Home];
T12 = [RecAc; P3Ac];
T15 = [P3Ac; Home];

%%--------------------------------------%%

Tray = T4;                               % Se escribe la trayectoria que se desea evaluar
qA = Tray(1, 1:4);
qB = Tray(2, 1:4);

for i=1:4
    qc_p(i) = (2*abs(qB(i)-qA(i)))/tf;   % velocidad de crucero
    tc(i) = tf/2;                        % tiempo de aceleracion
    qc_2p(i) = qc_p(i) ./ tc(i);         % aceleracion
end

dt = 1/50;                               % paso de tiempo (s)
t = 0:dt:tf;                             % vector de tiempo

n = length(qA);                          % número de articulaciones
N = numel(t);                            % número de pasos
Q   = zeros(n, N);                       % matriz de posiciones q(t)
Qd  = zeros(n, N);                       % matriz de velocidades q_p(t)
Qdd = zeros(n, N);                       % matriz de aceleraciones q_2p(t)

for i = 1:n
    qi = qA(i);
    qf = qB(i);
    qc = qc_p(i);
    acc = qc_2p(i);
    tci = tc(i);
    dq = qf - qi;
    s = sign(dq);
    if s == 0, s = 1; end

    % tiempo de crucero (puede ser 0 si perfil triangular)
    tcruise = tf - 2*tci;
    da = 0.5 * acc * tci^2;

    for k = 1:N
        tk = t(k);

        if abs(dq) < 1e-6 || tci == 0
            % sin movimiento
            q = qi;
            qdot = 0;
            qddot = 0;

        elseif tk <= tci
            % fase de aceleracion
            q = qi + s * 0.5 * acc * tk^2;
            qdot = s * acc * tk;
            qddot = s * acc;

        elseif tk <= (tci + tcruise)
            % fase de crucero
            q = qi + s * (da + qc * (tk - tci));
            qdot = s * qc;
            qddot = 0;

        elseif tk <= tf
            % fase de desaceleracion
            u = tf - tk;
            q = qf - s * 0.5 * acc * u^2;
            qdot = s * acc * u;
            qddot = -s * acc;
        else
            q = qf;
            qdot = 0;
            qddot = 0;
        end

        Q(i,k)   = q;
        Qd(i,k)  = qdot;
        Qdd(i,k) = qddot;
    end
end

% Resultados
disp('q(t) para cada articulacion:')
Q