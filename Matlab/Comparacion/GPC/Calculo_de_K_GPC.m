function [K1,K2,K3,K4] = Calculo_de_K_GPC(E,B,N,Nu,d)

% Constantes
g = 9.77412;

% Definición de la planta
G11a = tf(g,[1 0 0]);       % X(s) / Phi(s)
G22a = tf(-g,[1 0 0]);      % Y(s) / Theta(s)

G11b = tf(33.33,[1 0 0]);   % Phi(s) / T_phi(s)
G22b = tf(-33.33,[1 0 0]);  % Theta(s) / T_theta(s)

G11 = G11a * G11b;          % X(s) / T_phi(s)
G22 = G22a * G22b;          % Y(s) / T_theta(s)
G33 = tf(25,[1 0 0]);       % Z(s) / Fz(s)
G44 = tf(0.5587,[1 0 0]);   % Psi(s) / T_psi(s)

% Cambio de variable: como solo tenemos 4 actuadores, solo vamos
% a actuar en 4 variables (x,y,z,psi)

n_in = 4;

Ps = [G11   0    0    0;
       0   G22   0    0;
       0    0   G33   0;
       0    0    0   G44];

Ts = 0.01;                      % Tiempo de muestreo
Pz = c2d(Ps,Ts);                % Proceso discreto

[Bp,Ap,dp] = descompMPC(Pz);     % Descompone num, den y atraso de Pz en celdas


% Parametros de sintonia del GPC
% Encuentro cual es el retardo minimo en mi función de transferencia
dmin = [0 0 0 0]';
for i=1:n_in
    dmin(i) = min(dp(i,:));
end 

N = [140;140;140;140];                          % Ventana u Horizonte de Predicción
Nu = [120;120;120;120];                         % Horizonte de control
lambda = [1 1 1 1];                             % Ponderación de la acción de control
delta = [1 1 1 1];                              % Ponderación del seguimiento de referencia

% Matrices en bloque de los parametros de ponderación
% blkdiag(): Función de Matlab que toma cada valor y los ubica en una matriz diagonal
Ql=blkdiag(lambda(1)*eye(Nu(1)), lambda(2)*eye(Nu(2)), ...
           lambda(3)*eye(Nu(3)), lambda(4)*eye(Nu(4)) );  %(lambda -> Acción de Control)

Qd=blkdiag(delta(1)*eye(N(1)), delta(2)*eye(N(2)), ...
           delta(3)*eye(N(3)), delta(4)*eye(N(4)) );      %(delta  -> Seguimiento de Referencia)


% Cálculo de la ecuación Diofantina
%Se obtiene la matriz  y  (Numerador y denominador sistema MIMO). La matriz  es el minimo común denominador del sistema, es decir, 
[Bt,A] = BA_MIMO(Bp,Ap);

%%Cálculo de las Diofantinas para el sistema multivariable
%(Ecuación 6.5 Libro)
[E,En,F] = diophantineMIMO(A,N,dmin);

% Matriz de la Respuesta Forzada 
[G] = MatrizG(E,Bt,N,Nu,dp);

%Sistema sin Restricciones
M=inv(G'*Qd*G+Ql)*G'*Qd;    % Ganancia M
K1=M(1,:);                  % Tomo la primera columna unicamente (para control u1)
K2=M(Nu(1)+1,:);            % Tomo valores de M desde [N(1)+1] una muestra despues del primer 
                            % horizonte de control (para control u2)
K3=M(Nu(1)+Nu(2)+1,:);
K4=M(Nu(1)+Nu(2)+Nu(3)+1,:);

%K1 = K1(1);
%K2 = K2(2);
%K3 = K3(3);
%K4 = K4(4);