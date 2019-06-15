function xdot=dynamic_system(t,x,u)

m1=1.0;
m2=1.0;
l=10.0;
g=10.0;
 
A = [ (m1+m2)                 0.5*m2*l*cos(x(2))  
       0.5*m2*l*cos(x(2))      (1/3)*m2*(l^2)    ] ;

b= [ 0.5*m2*l*(x(4)^2)*sin(x(2))+ u
     0.5*m2*g*l*sin(x(2))];

y=A\b;

xdot=[x(3);x(4);y(1);y(2)];

end
