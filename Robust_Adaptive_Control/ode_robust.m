function [dz,T] = ode_robust(z,t,kp,kd,P,phi)
m1=1;m2=1;l1=1;l2=1;r1=0.45;r2=0.45;I1=0.084;I2=0.084;g=9.81;
m1_hat = 0.75;m2_hat = 0.75;I1_hat = 0.063;I2_hat = 0.063;
dz=zeros(6,1);
z=num2cell(z);
[t1, t2, t1_dot, t2_dot, u1, u2] = deal(z{:});

if abs(t1)>2*pi
    t1 = mod(t1, 2*pi);
end
if abs(t2)>2*pi
    t2 = mod(t2, 2*pi);
end

% x = [t1; t2; t1_dot; t2_dot];
q = [t1;t2];
q_dot = [t1_dot; t2_dot];
q_des = [(pi*t^3)/500 - (3*pi*t^2)/100 + pi;(pi*t^3)/1000 - (3*pi*t^2)/200 + pi/2];
q_dot_des = [(3*pi*t^2)/500 - (3*pi*t)/50;(3*pi*t^2)/1000 - (3*pi*t)/100];
q_ddot_des = [(3*pi*t)/250 - (3*pi/50);(3*pi*t/500)-(3*pi*t/100)];
% x_des = [q_des;q_dot_des];
x = [q-q_des;q_dot-q_dot_des];
e = [q-q_des];
e_dot = [q_dot-q_dot_des];
% vd = [(3*pi*t)/250 - (3*pi)/50; (3*pi*t)/500 - (3*pi)/100];

% M = [a+2*b*cos(t2), d+b*cos(t2); d+b*cos(t2), d];
% C = [-b*sin(t2)*t2_dot, -b*sin(t2)*(t1_dot+t2_dot); b*sin(t2)*t1_dot,0];
% G = [-m1*g*r1*sin(t1)-m2*g*(l1*sin(t1)+r2*sin(t1+t2)); -m2*g*r2*sin(t1+t2)];
% % where
% a = I1 + I2 + m1*r1^2 + m2*(l1^2 + r2^2);
% b = m2*l1*r2;
% d = I2 + m2*r2^2;
% % T = M*q_ddot + C*q_dot + G;
% 
a_hat = I1_hat + I2_hat + m1_hat*r1^2 + m2_hat*(l1^2 + r2^2);
b_hat = m2_hat*l1*r2;
d_hat = I2_hat + m2_hat*r2^2;
M_hat= [a_hat+2*b_hat*cos(t2), d_hat+b_hat*cos(t2); d_hat+b_hat*cos(t2), d_hat];
C_hat= [-b_hat*sin(t2)*t2_dot, -b_hat*sin(t2)*(t1_dot+t2_dot); b_hat*sin(t2)*t1_dot,0];
G_hat= [-m1_hat*g*r1*sin(t1)-m2_hat*g*(l1*sin(t1)+r2*sin(t1+t2)); -m2_hat*g*r2*sin(t1+t2)];
% where
% a_hat = I1_hat + I2_hat + m1_hat*r1^2 + m2_hat*(l1^2 + r2^2);
% b_hat = m2_hat*l1*r2;
% d_hat = I2_hat + m2_hat*r2^2;
% T = M_hat*q_ddot + C_hat*q_dot + G_hat;

Kn = [12	0	7	0;
        0	12	0	7];
% B = [0;0;1;1];
B = [0,0;0,0;1,0;0,1];
rho = eye(2).*11;

if phi>0
    if norm(x'*P*B)>phi
        vr = -((x'*P*B)/(norm(x'*P*B)))*rho;
    else
        vr = -((x'*P*B)/phi)*rho;
    end
else
    if norm(x'*P*B)~=0
        vr = -((x'*P*B)/(norm(x'*P*B)))*rho;
    else
        vr = 0;
    end
end

% vr = 0;

v = q_ddot_des - kp*e - kd*e_dot + vr';
T = M_hat*v + C_hat*q_dot + G_hat;

t1_ddot = T(1);
t2_ddot = T(2);
% v = -Kn*(e) + vd;
% u = M*v + C*q_dot + G;
u1 = T(1);
u2 = T(2);

dz(1) = t1_dot;
dz(2) = t2_dot;
%dz(3) = t1_ddot;
%dz(4) = t2_ddot;
dz(3) = (I2*u1 - I2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*r2^3*t1_dot^2*sin(t2) + l1*m2^2*r2^3*t2_dot^2*sin(t2) + g*l1*m2^2*r2^2*sin(t1) + I2*g*l1*m2*sin(t1) + I2*g*m1*r1*sin(t1) - l1*m2*r2*u2*cos(t2) + 2*l1*m2^2*r2^3*t1_dot*t2_dot*sin(t2) + l1^2*m2^2*r2^2*t1_dot^2*cos(t2)*sin(t2) - g*l1*m2^2*r2^2*sin(t1 + t2)*cos(t2) + I2*l1*m2*r2*t1_dot^2*sin(t2) + I2*l1*m2*r2*t2_dot^2*sin(t2) + g*m1*m2*r1*r2^2*sin(t1) + 2*I2*l1*m2*r2*t1_dot*t2_dot*sin(t2))/(- l1^2*m2^2*r2^2*cos(t2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dz(4) = -(I2*u1 - I1*u2 - I2*u2 - l1^2*m2*u2 - m1*r1^2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*r2^3*t1_dot^2*sin(t2) + l1^3*m2^2*r2*t1_dot^2*sin(t2) + l1*m2^2*r2^3*t2_dot^2*sin(t2) - g*l1^2*m2^2*r2*sin(t1 + t2) - I1*g*m2*r2*sin(t1 + t2) + g*l1*m2^2*r2^2*sin(t1) + I2*g*l1*m2*sin(t1) + I2*g*m1*r1*sin(t1) + l1*m2*r2*u1*cos(t2) - 2*l1*m2*r2*u2*cos(t2) + 2*l1*m2^2*r2^3*t1_dot*t2_dot*sin(t2) + 2*l1^2*m2^2*r2^2*t1_dot^2*cos(t2)*sin(t2) + l1^2*m2^2*r2^2*t2_dot^2*cos(t2)*sin(t2) - g*l1*m2^2*r2^2*sin(t1 + t2)*cos(t2) + g*l1^2*m2^2*r2*cos(t2)*sin(t1) - g*m1*m2*r1^2*r2*sin(t1 + t2) + I1*l1*m2*r2*t1_dot^2*sin(t2) + I2*l1*m2*r2*t1_dot^2*sin(t2) + I2*l1*m2*r2*t2_dot^2*sin(t2) + g*m1*m2*r1*r2^2*sin(t1) + 2*l1^2*m2^2*r2^2*t1_dot*t2_dot*cos(t2)*sin(t2) + l1*m1*m2*r1^2*r2*t1_dot^2*sin(t2) + 2*I2*l1*m2*r2*t1_dot*t2_dot*sin(t2) + g*l1*m1*m2*r1*r2*cos(t2)*sin(t1))/(- l1^2*m2^2*r2^2*cos(t2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dz(5) = u1;
dz(6) = u2;

end