
%1. the model of DFIG:
%% us=Gzs*is+Gzms*ir   ur=Gzr*ir+Gzmr*is
Gzs = [Rs+s*Ls -w*Ls; w*Ls Rs+s*Ls];
Gzr = [Rr+s*Lr -w2*Lr; w2*L2 Rr+s*Lr];
Gzms = [s*Lm -w*Lm; w*Lm s*Lm];
Gzmr = [s*Lm -w2*Lm; w2*Lm s*Lm];

%2. the model of RSC current controller
%% ur=Gci*(ir_ref-ir)+Gd1*ir+Gd2*is
Gci = [kp_ir+ki_ir/s 0; 0 kp_ir+ki_ir/s];
Gd1 = [0 -w2*Lm; w2*Lm 0];
Gd2 = [0 -w*Lm; w*Lm 0];

%3. the model of RSC power controller
%% ir_ref=Gcpq*(pq_ref-pq)+Gd3*us
Gcpq = [kp_pq+ki_pq/s 0; 0 kp_pq+ki_pq/s];
Gd3 =  [0 0; -1/w*Lm];

%% pq = Gpq_v*us+Gpq_i*is
Gpq_v = [Isd Isq; Isq -Isd];
Gpq_i = [Usd Usq; -Usq Usd];

%4. the model of PLL
tfpll = Kp_pll + Ki_pll/s;
Gpll = tfpll / (s + Usd*tfpll);
Gpll_vr = [0  Urq*Gpll;  0  -Urd*Gpll];
Gpll_ir = [0  Irq*Gpll;  0  -Ird*Gpll];
Gpll_is = [0  Isq*Gpll;  0  -Isd*Gpll];
Gpll_vs = [0  Usq*Gpll;  0  -Usd*Gpll];
Gpll_ul = [0  Ulq*Gpll;  0  -Uld*Gpll];
Gpll_il = [0  Ilq*Gpll;  0  -Ild*Gpll];
Gpll_vs = Gpll_vs+eye(2);

%5. the model of grid side filter
%% ul = us - Gzl*il
Gzl = [Rl+s*L -w*L; w*L Rl+s*L];

%6. the model of GSC current controller
%%  ul = Ggci*(il_ref-il)+Gdel*il
Ggci = -[kp_il+ki_il/s   0; 0  kp_il+ki_il/s];
Gdel = [0 -w*L; w*L 0];

%7. the model of GSC dc voltage loop 
%% il_ref = Ggcv*(udc_ref-udc)
Ggcv = [kp_dc+ki_dc/s   0; 0  kp_dc+ki_dc/s];

%8. the model of dc-link
%% Gpvl*ul+Gpil*il-Gpvr*ur-Gpir*ir = Gpdc*udc
Gpvl = [Ild Ilq; 0 0];
Gpil = [Uld Ulq; 0 0];
Gpvr = [Ird Irq; 0 0];
Gpir = [Urd Urq; 0 0];
Gpdc = [s*C*Udc 0; 0 0];

%9. the dc voltage disturb to ur and ul
Gdcr = [Urd/Udc Urq/Udc; 0 0];
Gdcl = [Uld/Udc Ulq/Udc; 0 0];

Gvsr = Gci*(Gd3*Gpll_vs - Gcpq*Gpq_v*Gpll_vs - Gcpq*Gpq_i*Gpll_is - Gpll_ir)+Gd1*Gpll_ir+Gd2*Gpll_is - Gpll_vr;
Gvsl = Ggci*Ggcv*Gst*Gpll_vs+(Gdel-Ggci)*Gpll_il - Gpll_vl;


A = zeros(12,12);
A(1:2,3:4) = Gzms;
A(1:2,5:6) = Gzs;
A(3:4,1:2) = eye(2);
A(3:4,3:4) = -Gzr;
A(3:4,5:6) = -Gzmr;
A(5:6,1:2) = eye(2);
A(5:6,1:2) = eye(2);
A(5:6,3:4) = Gci-Gd1;
A(5:6,5:6) = Gci*Gcpq*Gpq-Gd2;
A(5:6,11:12) = -Gdcr;
A(7:8,7:8)  = eye(2);
A(7:8,9:10)  = Gzl;
A(9:10,7:8)  = eye(2);
A(9:10,9:10)  = Ggci-Gdel;
A(9:10,11:12) = -Gdcl-Ggci*Ggcv;
A(11:12,1:2) = Gpvr;
A(11:12,3:4) = Gpir;
A(11:12,7:8) = -Gpvl;
A(11:12,9:10) = -Gpil;
A(11:12,11:12) = -Gpdc;

B = [eye(2); zeros(2); Gvsr; eye(2); Gvsl; zeros(2)];

X = A\B;
