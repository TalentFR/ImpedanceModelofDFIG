% The usdq in stator voltage orientated vector control
Usd = U_DFIG*sqrt(2/3);
Usq = 0;

%Calculate the static point of rotor side 
A = zeros(6,6);
A(1,1) = 1.5*Usd;
A(1,2) = 1.5*Usq;
A(2,1) = -1.5*Usq;
A(2,2) = 1.5*Usd;
A(3,1) = Rs;
A(3,2) = -w*Ls;
A(3,6) = -w*Lm;
A(4,1) = w*Ls;
A(4,2) = Rs;
A(4,5) = w*Lm;
A(5,2) = -w2*Lm;
A(5,3) = -1;
A(5,5) = Rr;
A(5,6) = -w2*Lr;
A(6,1) = w2*Lm;
A(6,4) = -1;
A(6,5) = w2*Lr;
A(6,6) = Rr;

B = [Ps;Qs;Usd;Usq;0;0;];

X = A\B;


Isd = X(1);
Isq = X(2);
Urd = X(3);
Urq = X(4);
Ird = X(5);
Irq = X(6);

% Calculate the static point of grid side
myfun = @(x)[x(1)+Rl*x(3)-Usd; x(2)+w*L*x(3)-Usq; 1.5*(x(1)*x(3))-1.5*(Urd*Ird+Urq*Irq)];
X2 = fsolve(myfun, [563;0;-100]);

Usd = Usd/Ubase;
Usq = Usq/Ubase;
Isd = X(1)/Ibase;
Isq = X(2)/Ibase;
Urd = X(3)/Ubase;
Urq = X(4)/Ubase;
Ird = X(5)/Ibase;
Irq = X(6)/Ibase;

Uld = X2(1)/Ubase;
Ulq = X2(2)/Ubase;
Ild = X2(3)/Ibase;
Ilq = 0/Ibase;   

Udc=1150/Udc_nom;
Ut = sqrt(Usd*Usd+Usq*Usq);

