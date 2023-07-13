Tutorial 3.1
Se creó la función lagd.m
% lagdmodel.m
clear
numd=[1 -0.1];
dend=conv([1 -0.8],[1 -0.9]);
N_sim=60;
k=0:(N_sim-1);
H=dimpulse(numd,dend,k);

% Generar funciones de Laguerre para a y N
a=0.8;
N=3;
[A1,L0]=lagd(a,N);
L(:,1)=L0;
for kk=2:N_sim
    L(:,kk)=A1*L(:,kk-1);
end

% Calculo de los coeficientes de Laguerre
c1=L(1,:)*H; % Aquí hay problema, L debería tener o 59 columnas o H deberia tener 60 Filas
c2=L(2,:)*H; % Deben coincidir la dimensión de las columnas de L con la dimensión de las filas de H
c3=L(3,:)*H;
H_model=c1*L(1,:)+c2*L(2,:)+c3*L(3,:);

figure
plot(k(1:59),H)
hold on
plot(k,H_model,'LineWidth',2,'Color',[.8 0 0])
set(gca,'FontSize',20,'FontName','helvetica');
legend('data','model')
xlabel('Sampling Instant')
ylabel('Impulse Response')