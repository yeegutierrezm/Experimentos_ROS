function [u1,y1,udot1,t]=cssimuob(xm,u,y,sp,Ap,Bp,Cp,A,B,C,N_sim,Omega,Psi,K_ob,Lzerot,h)
[m1,n1]=size(Cp);
[n1,n_in]=size(Bp);
X_hat=zeros(n1+m1,1);

for kk=1:N_sim
    Xsp=[zeros(n1,1);sp(:,kk)];
    eta=-(Omega\Psi)*(X_hat-Xsp);
    udot=Lzerot*eta;
    u=u+udot*h;
    
    udot1(1:n_in,kk)=udot;
    u1(1:n_in,kk)=u;
    y1(1:m1,kk)=y;
    X_hat=X_hat+(A*X_hat+K_ob*(y-C*X_hat))*h+B*udot*h;
    
    xm=xm+(Ap*xm+Bp*u)*h;
    y=Cp*xm;
end
t=0:h:(N_sim-1)*h;
%y=Cp*xm+E*epsion(kk);