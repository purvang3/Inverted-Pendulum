function [A,B] = linearize(Fname, x0, u0)


n = size(x0) ;
m = size(u0) ;
delta = 0.001 ;
x = x0 ;
u = u0 ;
t = 0.0 ;

for j=1:n
  x(j) = x(j) + delta;
  A(:,j) = ( feval(Fname,t,x,u) - feval(Fname,t,x0,u0))/delta ;
  x(j) = x(j) - delta ;
end
for j=1:m
  u(j) = u(j) + delta;
  B(:,j) = (feval(Fname,t,x,u)-feval(Fname,t,x0,u0))/delta ;
  u(j) = u(j) - delta ;
end