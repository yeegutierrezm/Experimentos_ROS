function X=Iint(A,p,Y)
[n,N]=size(Y);
X=zeros(n,N);
Ai=inv(A+p*eye(n,n));
X(:,1)=Ai*Y(:,1);
alpha=-2*p*X(:,1);
for i=2:N
    X(:,i)=Ai*(Y(:,i)+alpha);
    alpha=alpha-2*p*X(:,i);
end