function [Mu,Mu1]=Mucon(p,N,n_in,delta_t,tau)
%function for generating matrix M for
%the constraints on the control signal
%constraints are imposed on the zero time and tau time
%delta-t is the sampling interval
%Mu is for constraints to be imposed on the zero sample
%Mu1 is for constraints to be imposed on tau time
N_pa=sum(N);
k0=1;
[Al,L0]=lagc(p(k0),N(k0));
L_t=zeros(n_in,N_pa);
L_t(1,1:N(1))=L0';
cc=N(1);
for k0=2:n_in
    [Al,L0]=lagc(p(k0),N(k0));
    L_t(k0,cc+1:cc+N(k0))=L0';
    cc=cc+N(k0);
end
% constraints on second sample
k0=1;
[Al,L0]=lagc(p(k0),N(k0));
L1=expm(Al*tau)*L0;
L_t1=zeros(n_in,N_pa);
L_t1(1,1:N(1))=(L1'-L0')*inv(Al')+L0'*delta_t;
cc=N(1);

for k0=2:n_in
    [Al,L0]=lagc(p(k0),N(k0));
    L1=expm(Al*tau)*L0;
    L_t1(k0,cc+1:cc+N(k0))=(L1'-L0')*inv(Al')+L0'*delta_t;
    cc=cc+N(k0);
end
Mu=[L_t*delta_t];
Mu1=[L_t1];