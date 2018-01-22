% In this model, the variable in controllers are per unit. 

%1. the model of DFIG:
%% us=Gzs*is+Gzms*ir   ur=Gzr*ir+Gzmr*is
Gzs = [Rs+s*Ls -w*Ls; w*Ls Rs+s*Ls];
Gzr = [Rr+s*Lr -w2*Lr; w2*Lr Rr+s*Lr];
Gzms = [s*Lm -w*Lm; w*Lm s*Lm];
Gzmr = [s*Lm -w2*Lm; w2*Lm  s*Lm];

%2. the model of RSC current controller
%% ur=Gci*(ir_ref-ir)+Gd1*ir+Gd2*is
Gci = [kp_ir+ki_ir/s 0; 0 kp_ir+ki_ir/s];
Gd1 = [0 -w2*Lr; w2*Lr 0]/Zbase;
Gd2 = [0 -w2*Lm; w2*Lm 0]/Zbase;

%3. the model of RSC power controller
%% ir_ref=Gcpq*(pq_ref-pq)+Gd3*us
Gcpq = [kp_pq+ki_pq/s 0; 0 kp_pq+ki_pq/s];
Gd3 =  [0 0; -1/1.3*w*Ls 0]*Zbase;

%% pq = Gpq_v*us+Gpq_i*is
Gpq_v = -[Isd Isq; Isq -Isd];
Gpq_i = -[Usd Usq; -Usq Usd];

%4. the model of PLL
%% x_s = x_c + Gpll_x*us
tfpll = kp_pll + ki_pll/s;
Gpll = tfpll / (s + Usd*tfpll);
Gpll_ur = [0  Urq*Gpll;  0  -Urd*Gpll];
Gpll_ir = [0  Irq*Gpll;  0  -Ird*Gpll];
Gpll_is = [0  Isq*Gpll;  0  -Isd*Gpll];
Gpll_us = [0  Usq*Gpll;  0  -Usd*Gpll];
Gpll_ul = [0  Ulq*Gpll;  0  -Uld*Gpll];
Gpll_il = [0  Ilq*Gpll;  0  -Ild*Gpll];
Gpll_us = Gpll_us+eye(2);

%5. the model of grid side filter
%% ul = us - Gzl*il
Gzl = [Rl+s*L -w*L; w*L Rl+s*L];

%6. the model of GSC current controller
%%  ul = Ggci*(il_ref-il)+Gdel*il
Ggci = -[kp_il+ki_il/s   0; 0  kp_il+ki_il/s];
Gdel = [0 -w*L; w*L 0]/Zbase;

%7. the model of GSC dc voltage loop 
%% il_ref = Ggcv*(udc_ref-udc)
Ggcv = [kp_dc+ki_dc/s   0; 0 0];

%8. the model of dc-link
%% Gpvl*ul+Gpil*il-Gpvr*ur-Gpir*ir = Gpdc*udc
Gpvl = 1.5*[Ild Ilq; 0 0]*Ibase;
Gpil = 1.5*[Uld Ulq; 0 0]*Ubase;
Gpvr = 1.5*[Ird Irq; 0 0]*Ibase;
Gpir = 1.5*[Urd Urq; 0 0]*Ubase;
Gpdc = [s*C*Udc*Udc_nom 0; 0 0];

%9. the dc voltage disturb to ur and ul
m=2*Ubase/Udc_nom;
Gdcr = m*[Urd/Udc Urq/Udc; 0 0];
Gdcl = m*[Uld/Udc Ulq/Udc; 0 0];

Gvsr = Gci*(Gd3*Gpll_us - Gcpq*Gpq_v*Gpll_us - Gcpq*Gpq_i*Gpll_is - Gpll_ir)+Gd1*Gpll_ir+Gd2*Gpll_is - Gpll_ur;
Gvsl = (Gdel-Ggci)*Gpll_il - Gpll_ul;

% A = sym(zeros(12,12));
% A(1:2,3:4) = Gzms;
% A(1:2,5:6) = Gzs;
% A(3:4,1:2) = eye(2);
% A(3:4,3:4) = -Gzr;
% A(3:4,5:6) = -Gzmr;
% A(5:6,1:2) = eye(2)/Ubase;
% A(5:6,3:4) = (Gci-Gd1)/Ibase;
% A(5:6,5:6) = (Gci*Gcpq*Gpq_i-Gd2)/Ibase;
% A(5:6,11:12) = -Gdcr/Udc_nom;
% A(7:8,7:8)  = eye(2);
% A(7:8,9:10)  = Gzl;
% A(9:10,7:8)  = eye(2)/Ubase;
% A(9:10,9:10)  = (Ggci-Gdel)/Ibase;
% A(9:10,11:12) = (-Gdcl+Ggci*Ggcv)/Udc_nom;
% A(11:12,1:2) = Gpvr;
% A(11:12,3:4) = Gpir;
% A(11:12,7:8) = -Gpvl;
% A(11:12,9:10) = -Gpil;
% A(11:12,11:12) = -Gpdc;

%10. The  whole impedance model of DFIG system
%% AX=B, X=[ur, ir, is, ul, il, udc]';
Zero = zeros(2,2);
E = eye(2);
A = [     Zero       Gzms                 Gzs                  Zero            Zero             Zero;
            E         -Gzr                -Gzmr                 Zero            Zero             Zero;
          E/Ubase  (Gci-Gd1)/Ibase  (Gci*Gcpq*Gpq_i-Gd2)/Ibase  Zero            Zero         -Gdcr/Udc_nom;
           Zero       Zero                Zero                    E              Gzl             Zero;
           Zero       Zero                Zero                 E/Ubase      (Ggci-Gdel)/Ibase  (Ggci*Ggcv-Gdcl)/Udc_nom;
           Gpvr       Gpir                Zero                 -Gpvl            -Gpil            Gpdc];

B = [eye(2); zeros(2); Gvsr/Ubase; eye(2); Gvsl/Ubase; zeros(2)];

%11. solve the impedance model
X =A\B;
Ydfig_rsc=X(5:6,:);
Ygsc = X(9:10,:);
Ysys = Ydfig_rsc + Ygsc;