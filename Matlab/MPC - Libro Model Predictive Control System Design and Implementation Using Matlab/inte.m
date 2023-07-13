function v=inte(F,h)
%Use Simpson’s Rule to compute an integral expression
% F is the discretized function
% h is the sampling interval
% v is the computed value
[m,n]=size(F);
if m==1 m=n;
end
if n==1 n=m; end
NN=(n-1)/2;
s=0;
for j=2:2:2*NN
s=s+4*F(j);
end
for j=3:2:2*NN-1
s=s+2*F(j);
end
v=(s+F(1)+F(2*NN+1))*h/3;