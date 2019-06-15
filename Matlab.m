clear all;
clc;

m1=1;  %mass of cart
m2=1;  %mass of pendulum
l=10;
g=10;


x0 = [ 0.0  0.0  0.0  0.0 ]' ;  % Initial States

u0 = [ 0.0 ]' ;   % Initial Control Force

[A,B] = linearize('dynamic_system', x0, u0);  % Linearizing Non linear Characteristic of Inverted Pendulum
                                              % A and B respresents Dynamics of
                                              % system
C = [ 1.0 0.0 0.0 0.0 ];   % Output measurenment variable is cart position
%C = [ 0.0 1.0 0.0 0.0 ];   % Output measurenment variable is angle and
                            %system will become unObservable


% Sampling Period:

T_sample = 0.0001 ;

% ZOH equivalent discrete-time LTI model

SYS1 = ss(A,B,C,0) ;
SYS1D = c2d(SYS1,T_sample,'zoh') ;  % Descrete values for A,B,C using sampling time and zero order hold method
[A_d,B_d,C_d,D_d] =ssdata(SYS1D);   % getting Values 


% Controllability and observability of the discrete time system

Wc = ctrb(A_d,B_d) ;  % Checking Controllability
nc = rank(Wc) ;
if nc==4 
   disp('Discrete time model: Controllable ');
else
   disp('Discrete time model: Not Controllable') ; 
end


Wo = obsv(A_d,C_d) ;   % Checking Observability
no = rank(Wo) ;
if no==4 
  disp('Discrete time model: Observable');
else 
   disp('Discrete time model: Not Observable') ; 
end

%Augmented Linear System Dunamics: to include integral control

v0=[0;0;0;0];

A_da=[A_d, v0;  
      C_d, 1];   % Including Integral action using Augmented matrix

B_da=[B_d;
       0];

C_da=[C_d, 0];

% Desired closed loop eigenvalues;

%p_c = [ -1+j*1  -1-j*1  -4+j*4  -4-j*4 ] ;
p_c = 5*[ -1+j*1  -1-j*1  -4+j*4  -4-j*4 ] 
p_cd = exp(p_c*T_sample) ; % desired poles in z-domain
%p_ca = [ -1+j*1 -1-j*1 -4+j*4 -4-j*4 -0.5 ] ;
p_ca = 5*+[ -1+j*1 -1-j*1 -4+j*4 -4-j*4 -0.5 ] ;
p_cad = exp(p_ca*T_sample) ; % desired poles in z-domain

% Desired full-state observer (state estimator) poles:

%p_e =5*p_c;
p_e = [-1+j*1  -1-j*1  -4+j*4  -4-j*4 ];
p_ed = exp(p_e *T_sample) ;

K_d = place(A_d,B_d,p_cd) ;
L_d = (place(A_d',C',p_ed))' ;
K_da = place(A_da, B_da, p_cad) ;




choice= 0 ;
while (~((choice==1 || choice==2)))
 choice = input('Enter \n 1 for without integral control \n 2 for with integral control simulation: ');
end
if (choice==1)
  K = K_d ;
  K_i = 0.0 ;
elseif (choice==2)
  K=K_da(1:4);
  K_i = K_da(5) ;
end



% Feedforward gain to achieve uniy DC gain
% Feed for9ward gain:

 I = eye(4,4);
 N_d = 1/(C_d*inv(I-A_d+B_d*K)*B_d ) ;

 