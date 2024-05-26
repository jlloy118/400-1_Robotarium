close all
clear all
syms q qdot alphai betai wi ki Gp1i Gp2i delta qfront qdotfront qback qdotback Kpi Gpi...
    Giv Gp0 delta0 qdot0 q0 Giv0 Kp1 Kp2 Kp0 Kv0 Kiv wbar qref qdotref epsilon lemmaifront t lemmaiback Xisym real  

xi = sym('xi','real');
xref = sym('xref');
lemmai = sym('lemmai');
lemmaidot = sym('lemmaidot');
gifront = sym('gifront');
giback = sym('giback');
gi0 = sym('gi0');
hifront = sym('hifront');
hiback = sym('hiback');
hi0 = sym('hi0');
Xip = sym('Xi');
Ti = sym('Ti');
Ai = sym('Ai');
Bi = sym('Bi');

Gp2i = Gp1i;
Kp2 = Kp1;


Kp1 = sqrt(Kpi);
Gp1i = sqrt(Gpi);

xref = [qref;qdotref];


% Upper and lower bounds for tanh function
tanh_upper = 1;
tanh_lower = -1;

% Original equations
gifront = Gp1i*tanh(Gp2i*(qfront - q - delta)) + Giv*(qdotfront - qdot);
giback = Gp1i*tanh(Gp2i*(qback - q + delta)) + Giv*(qdotback - qdot);
gi0 = Gp0*(q0 - q - delta0) + Giv0*(qdot0 - qdot);

hifront = Kp1*tanh(Kp2*(qfront - q - delta)) + Kiv*(qdotfront - qdot);
hiback = Kp1*tanh(Kp2*(qback - q + delta)) + Kiv*(qdotback - qdot);
hi0 = Kp0*(q0 - q - delta0) + Kv0*(qdot0 - qdot);

% Applying upper and lower bounds to the tanh function
% Control inputs with upper bound tanh replaced
gifront_upper = subs(gifront, tanh(Gp2i*(qfront - q - delta)), tanh_upper);
giback_upper = subs(giback, tanh(Gp2i*(qback - q + delta)), tanh_upper);
hifront_upper = subs(hifront, tanh(Kp2*(qfront - q - delta)), tanh_upper);
hiback_upper = subs(hiback, tanh(Kp2*(qback - q + delta)), tanh_upper);

% Control inputs with lower bound tanh replaced
gifront_lower = subs(gifront, tanh(Gp2i*(qfront - q - delta)), tanh_lower);
giback_lower = subs(giback, tanh(Gp2i*(qback - q + delta)), tanh_lower);
hifront_lower = subs(hifront, tanh(Kp2*(qfront - q - delta)), tanh_lower);
hiback_lower = subs(hiback, tanh(Kp2*(qback - q + delta)), tanh_lower);

% Compute lemmai derivatives and control inputs using upper and lower bounds
lemmaidot_upper = gifront_upper + epsilon*giback_upper + gi0;
Vxi_upper = hifront_upper + epsilon*hiback_upper + hi0 + ki*Xip;

lemmaidot_lower = gifront_lower + epsilon*giback_lower + gi0;
Vxi_lower = hifront_lower + epsilon*hiback_lower + hi0 + ki*Xip;

xi = q;
lemmaidot = gifront + epsilon*giback + gi0;
Xi = lemmai + (ki^-1)*wbar;
Vxi = hifront + epsilon*hiback + hi0 + ki*Xi;
VxiU = hifront_upper + epsilon*hiback_upper + hi0 + ki*Xi;
VxiL = hifront_lower + epsilon*hiback_lower + hi0 + ki*Xi;

Vlemmai = gifront + epsilon*giback + gi0;
VlemmaiU = gifront_upper + epsilon*giback_upper + gi0;
VlemmaiL = gifront_lower + epsilon*giback_lower + gi0;
pi = [wi;0];
zi = [xi;Xisym];


vi = [Vxi;Vlemmai];
viU = [VxiU;VlemmaiU];
viL = [VxiL;VlemmaiL];

zref = [xref;0];

F = [1 0; 0 0];
zdot = F*zi + vi + pi;
Ai = alphai*eye(1);
Bi = betai(eye(1)); %Not used for now
Ti = [1 Ai ;
      Bi 1];

Tinv = Ti^-1;

vbar = Ti*vi;
vbarU = Ti*viU;
vbarL = Ti*viL;

%vbardelz = [diff(vbar(1),zi(1));1]; %check if 2nd element would actually be one
vbardelz = jacobian(vbar,zi);
%vbardelzU = [diff(vbarU(1),zi(1));1];
vbardelzU = jacobian(vbarU,zi);
%vbardelzL = [diff(vbarL(1),zi(1));1];
vbardelzL = jacobian(vbarL,zi);


disp(vbardelz)

%Jii = Ti*F*(Ti)^-1 + [diff(vbar(1),zi(1)).*Tinv(:,1),1.*Tinv(:,2)];
Jii = Ti*F*(Ti)^-1 + jacobian(vbar,zi)*Ti^-1;
%JiiU = Ti*F*(Ti)^-1 + [diff(vbarU(1),zi(1)).*Tinv(:,1),1.*Tinv(:,2)];
JiiU = Ti*F*(Ti)^-1 + jacobian(vbarU,zi)*Ti^-1;
%JiiL = Ti*F*(Ti)^-1 + [diff(vbarL(1),zi(1))*Tinv(:,1),1.*Tinv(:,2)];
JiiL = Ti*F*(Ti)^-1 + jacobian(vbarL,zi)*Ti^-1;

%% WIP
Hiback = [hiback;giback];
HibackU = [hiback_upper;giback_upper];
HibackL = [hiback_lower;giback_lower];

Hifront = [hifront;gifront];
HifrontU = [hifront_upper;gifront_upper];
HifrontL = [hifront_lower;gifront_lower];

Xifront = lemmaifront + ki^-1*wbar;
zifront = [qfront;Xifront];
Xiback = lemmaiback + ki^-1*wbar;
ziback = [qback;Xiback];

Hfrontbar = Ti*Hifront;
HfrontbarU = Ti*HifrontU;
HfrontbarL = Ti*HifrontL;

Hbackbar = Ti*Hiback;
HbackbarU = Ti*HibackU;
HbackbarL = Ti*HibackL;


% Jiim1 = Ti*[[diff(Hfrontbar(1),zifront(1));0].*Tinv(:,1), [diff(Hfrontbar(1),zifront(1))].*Tinv(:,2)];
% Jiim1U = Ti*[[diff(HfrontbarU(1),zifront(1));0].*Tinv(:,1), [diff(HfrontbarU(1),zifront(1))].*Tinv(:,2)];
% Jiim1L = Ti*[[diff(HfrontbarL(1),zifront(1));0].*Tinv(:,1), [diff(HfrontbarL(1),zifront(1))].*Tinv(:,2)];
% 
% Jiip1  = [[diff(Hbackbar(1),ziback(1));0].*Tinv(:,1), [diff(Hbackbar(1),ziback(1));0].*Tinv(:,2)];
% Jiip1U = [[diff(HbackbarU(1),ziback(1));0].*Tinv(:,1), [diff(HbackbarU(1),ziback(1));0].*Tinv(:,2)];
% Jiip1L = [[diff(HbackbarL(1),ziback(1));0].*Tinv(:,1), [diff(HbackbarL(1),ziback(1));0].*Tinv(:,2)];
Jiim1 = Ti * jacobian(Hifront,zi) * Ti^-1;
Jiim1U = Ti * jacobian(HifrontU,zi) * Ti^-1;
Jiim1L = Ti * jacobian(HifrontL,zi) * Ti^-1;

Jiip1 = Ti * jacobian(Hifront,zi) * Ti^-1;
Jiip1U = Ti*jacobian(HibackU,zi) * Ti^-1;
Jiip1L = Ti*jacobian(HibackL,zi) * Ti^-1;
%%

zibar = Ti*zi;
zidelzbar1 = [1;alphai^-1];
zidelzbar2 = [0;1];

Jiibar = Ti*F*(Ti^-1) + [vbardelz*zidelzbar1, vbardelz*zidelzbar2;];
JiibarU = Ti*F*(Ti^-1) + [vbardelzU*zidelzbar1, vbardelzU*zidelzbar2;];
JiibarL = Ti*F*(Ti^-1) + [vbardelzL*zidelzbar1, vbardelzL*zidelzbar2;];
 
% Jii = [alpha*(Gp1i*Gp2i*(tanh(Gp2i*(delta + q - qfront))^2 - 1) - Gp0 + Gp1i*Gp2i*epsilon*(tanh(Gp2i*(delta - q + qback))^2 - 1)) - Kp0 + Kp1*Kp2*(tanh(Kp2*(delta + q - qfront))^2 - 1) + Kp1*Kp2*epsilon*(tanh(Kp2*(delta - q + qback))^2 - 1) + 1,                   alpha*(Gp1i*Gp2i*(tanh(Gp2i*(delta + q - qfront))^2 - 1) - Gp0 + Gp1i*Gp2i*epsilon*(tanh(Gp2i*(delta - q + qback))^2 - 1)) - Kp0 + Kp1*Kp2*(tanh(Kp2*(delta + q - qfront))^2 - 1) + Kp1*Kp2*epsilon*(tanh(Kp2*(delta - q + qback))^2 - 1)...
% ;1 - conj(alpha)*(alpha*(Gp1i*Gp2i*(tanh(Gp2i*(delta + q - qfront))^2 - 1) - Gp0 + Gp1i*Gp2i*epsilon*(tanh(Gp2i*(delta - q + qback))^2 - 1)) - Kp0 + Kp1*Kp2*(tanh(Kp2*(delta + q - qfront))^2 - 1) + Kp1*Kp2*epsilon*(tanh(Kp2*(delta - q + qback))^2 - 1)) - conj(alpha), 1 - conj(alpha)*(alpha*(Gp1i*Gp2i*(tanh(Gp2i*(delta + q - qfront))^2 - 1) - Gp0 + Gp1i*Gp2i*epsilon*(tanh(Gp2i*(delta - q + qback))^2 - 1)) - Kp0 + Kp1*Kp2*(tanh(Kp2*(delta + q - qfront))^2 - 1) + Kp1*Kp2*epsilon*(tanh(Kp2*(delta - q + qback))^2 - 1))];
% 








