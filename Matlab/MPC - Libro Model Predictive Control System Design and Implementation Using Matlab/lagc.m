function [Ap,L0]=lagc(p,N)
%Generating system matrix Ap
Ap=-p*eye(N,N);
for ii=1:N
for jj=1:N
if jj<ii, Ap(ii,jj)=-2*p;
end
end
end
L0=sqrt(2*p)*ones(N,1);