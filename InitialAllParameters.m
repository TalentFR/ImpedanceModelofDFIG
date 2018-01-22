%parameters of DFIG
S_DFIG = 1.5e6;     %VA
U_DFIG = 690;     	%V

Zbase = U_DFIG*U_DFIG/S_DFIG;
% Ubase = U_DFIG*sqrt(2/3);
% Ibase = S_DFIG/U_DFIG*sqrt(2/3);

Ubase = 563;
Ibase = 1775;
wb = 100*pi;

w = 100*pi;
Llr = 0.16*Zbase/100/pi;   %H
Lls = 0.18*Zbase/100/pi;
Lm = 2.9*Zbase/100/pi;
Rr = 0.016*Zbase;         %oumu
Rs = 0.023*Zbase;
Ls = Lm+Lls;
Lr = Lm+Llr;

wr = 130*pi;
Ps = -1.2e6;
Qs = 0;
w2 = w-wr;

%parameters of grid-side converter
% L = 3.309e-4;
% Rl =0.0095;

% L = 1*Zbase/100/pi;
% Rl = 0.02*Zbase;
L = 1.0e-3;
Rl = 0.0063;


%parameters of dc-link
C=1e-2; %F
Udc_nom = 1150;

%parameters of grid
% Lg = 7.5774e-4;     %SCR=1.5
% Rg = 0.0340;

Lg = 5.0516e-4;    %SCR = 2;
Rg = 0.02;
Cg = 0.2006;

%%%%--------parameters of control system---------%%%%%
%parameters of rotor-side controllers
kp_ir =1.2;
ki_ir = 10;
kp_pll = 100;
ki_pll = 1000;
kp_pq  =1;
ki_pq = 10;

Ts = 0;

%parameters of grid-side controllers
kp_il =10;
ki_il = 100;
kp_pll2 = 100;
ki_pll2 = 1000;
kp_dc =6;
ki_dc = 400;
kp_ac = 0;
ki_ac = 0;
