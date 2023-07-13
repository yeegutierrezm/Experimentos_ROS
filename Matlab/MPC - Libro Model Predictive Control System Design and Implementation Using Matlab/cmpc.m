function [Omega,Psi]=cmpc(A,B,p,N,Tp,Q,R)
[n,n_in]= size(B);
tau_del=0.001/max(p);
Tpm=max(Tp);
tau=0:tau_del:Tpm;
Np=length(tau);
N_pa=sum(N);
Omega=zeros(N_pa,N_pa);
Psi=zeros(N_pa,n);
S_in=zeros(n,N_pa);

R_L=eye(N_pa,N_pa);
kk=1;
for i=1:n_in
    R_L(kk:kk-1+N(i),kk:kk-1+N(i))=R(i,i)*R_L(kk:kk-1+N(i),kk:kk-1+N(i));
    kk=kk+N(i);
end

[Al,L0]=lagc(p(1),N(1));
Eae=expm(A*tau_del);
Eap=expm(Al*tau_del);
L=Eap*L0;
Y=-B(:,1)*L'+Eae*B(:,1)*L0';
X=Iint(A,p(1),Y);
S_in(:,1:N(1))=X;
In_s=1;
for jj=2:n_in
    [Al,L0]=lagc(p(jj),N(jj));
    Eap=expm(Al*tau_del);
    L=Eap*L0;
    Y=-B(:,jj)*L'+Eae*B(:,jj)*L0';
    X=Iint(A,p(jj),Y);
    In_s=N(jj-1)+In_s;
    In_e=In_s+N(jj)-1;
    S_in(:,In_s:In_e)=X;
end
S_sum=S_in;

for i=2:Np-1
    kk=1;
    [Al,L0]=lagc(p(kk),N(kk));
    Eap=expm(Al*tau_del);
    S_sum(:,1:N(kk))=Eae*S_sum(:,1:N(kk))+S_in(:,1:N(kk))*(Eap^(i-1))';
    In_s=1;
    for kk=2:n_in
        [Al,L0]=lagc(p(kk),N(kk));
        Eap=expm(Al*tau_del);
        In_s=N(kk-1)+In_s;
        In_e=In_s+N(kk)-1;
        S_sum(:,In_s:In_e)=Eae*S_sum(:,In_s:In_e)+S_in(:,In_s:In_e)*(Eap^(i-1))';
    end
    phi=S_sum;
    Omega=Omega+phi'*Q*phi;
    Psi=Psi+phi'*Q*Eae^i;
end

Omega=Omega*tau_del+R_L;
Psi=Psi*tau_del;