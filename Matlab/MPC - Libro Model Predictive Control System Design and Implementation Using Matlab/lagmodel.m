p=1;
N=4;
delta_t=0.01;
Tm=8;
N_sample=Tm/delta_t;
t=0:delta_t:(N_sample-1)*delta_t;
[Ap,L0]=lagc(p,N);
for i=1:N_sample
    L(:,i)=expm(Ap*t(i))*L0;
end

