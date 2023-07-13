function [M_dv,Lzerot]=Mder(p,N,n_in,tau)
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

Lzerot=L_t;
k0=1;
[Al,L0]=lagc(p(k0),N(k0));
L1=expm(Al*tau)*L0;
L_t1=zeros(n_in,N_pa);
L_t1(1,1:N(1))=L1';
cc=N(1);
for k0=2:n_in
    [Al,L0]=lagc(p(k0),N(k0));
    L1=expm(Al*tau)*L0;
    L_t1(k0,cc+1:cc+N(k0))=L1';
    cc=cc+N(k0);
end
M_dv=[L_t1];