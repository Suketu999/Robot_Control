function [dz,T] = ode_adaptive(z,t,kp,kd,P,Gamma)

m1=1;m2=1;l1=1;l2=1;r1=0.45;r2=0.45;I1=0.084;I2=0.084;g=9.81;
% m1_hat = 0.75;m2_hat = 0.75;I1_hat = 0.063;I2_hat = 0.063;
dz=zeros(11,1);
z=num2cell(z);
[t1, t2, t1_dot, t2_dot, alpha_hat(1,1), alpha_hat(2,1), alpha_hat(3,1), alpha_hat(4,1), alpha_hat(5,1), T(1), T(2)] = deal(z{:});

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
% a_hat = I1_hat + I2_hat + m1_hat*r1^2 + m2_hat*(l1^2 + r2^2);
% b_hat = m2_hat*l1*r2;
% d_hat = I2_hat + m2_hat*r2^2;
% M_hat= [a_hat+2*b_hat*cos(t2), d_hat+b_hat*cos(t2); d_hat+b_hat*cos(t2), d_hat];
% C_hat= [-b_hat*sin(t2)*t2_dot, -b_hat*sin(t2)*(t1_dot+t2_dot); b_hat*sin(t2)*t1_dot,0];
% G_hat= [-m1_hat*g*r1*sin(t1)-m2_hat*g*(l1*sin(t1)+r2*sin(t1+t2)); -m2_hat*g*r2*sin(t1+t2)];
% where
% a_hat = I1_hat + I2_hat + m1_hat*r1^2 + m2_hat*(l1^2 + r2^2);
% b_hat = m2_hat*l1*r2;
% d_hat = I2_hat + m2_hat*r2^2;
% T = M_hat*q_ddot + C_hat*q_dot + G_hat;

Kn = [12	0	7	0;
        0	12	0	7];
% B = [0;0;1;1];
B = [0,0;0,0;1,0;0,1];
% rho = eye(2).*11;

% if phi>0
%     if norm(x'*P*B)>phi
%         vr = -((x'*P*B)/(norm(x'*P*B)))*rho;
%     else
%         vr = -((x'*P*B)/phi)*rho;
%     end
% else
%     if norm(x'*P*B)~=0
%         vr = -((x'*P*B)/(norm(x'*P*B)))*rho;
%     else
%         vr = 0;
%     end
% end
v = q_ddot_des - Kn*x;
%alpha_hat = alpha_hat';

Yo = [v(1), cos(t2)*(2*v(1) + v(2)) - 2*sin(t2)*t1_dot*t2_dot - sin(t2)*t2_dot^2, v(2), -sin(t1)*g, -sin(t1 + t2)*g;
    0, sin(t2)*t1_dot^2 + cos(t2)*v(1), v(1) + v(2), 0, -sin(t1+t2)*g];

% vr = 0;
% v = q_ddot_des - kp*e - kd*e_dot + vr';

T = Yo*alpha_hat;
% T = M_hat*v + C_hat*q_dot + G_hat;
u1 = T(1);
u2 = T(2);

dz(1) = t1_dot;
dz(2) = t2_dot;
t1_ddot = (I2*u1 - I2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*r2^3*t1_dot^2*sin(t2) + l1*m2^2*r2^3*t2_dot^2*sin(t2) + g*l1*m2^2*r2^2*sin(t1) + I2*g*l1*m2*sin(t1) + I2*g*m1*r1*sin(t1) - l1*m2*r2*u2*cos(t2) + 2*l1*m2^2*r2^3*t1_dot*t2_dot*sin(t2) + l1^2*m2^2*r2^2*t1_dot^2*cos(t2)*sin(t2) - g*l1*m2^2*r2^2*sin(t1 + t2)*cos(t2) + I2*l1*m2*r2*t1_dot^2*sin(t2) + I2*l1*m2*r2*t2_dot^2*sin(t2) + g*m1*m2*r1*r2^2*sin(t1) + 2*I2*l1*m2*r2*t1_dot*t2_dot*sin(t2))/(- l1^2*m2^2*r2^2*cos(t2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
t2_ddot = -(I2*u1 - I1*u2 - I2*u2 - l1^2*m2*u2 - m1*r1^2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*r2^3*t1_dot^2*sin(t2) + l1^3*m2^2*r2*t1_dot^2*sin(t2) + l1*m2^2*r2^3*t2_dot^2*sin(t2) - g*l1^2*m2^2*r2*sin(t1 + t2) - I1*g*m2*r2*sin(t1 + t2) + g*l1*m2^2*r2^2*sin(t1) + I2*g*l1*m2*sin(t1) + I2*g*m1*r1*sin(t1) + l1*m2*r2*u1*cos(t2) - 2*l1*m2*r2*u2*cos(t2) + 2*l1*m2^2*r2^3*t1_dot*t2_dot*sin(t2) + 2*l1^2*m2^2*r2^2*t1_dot^2*cos(t2)*sin(t2) + l1^2*m2^2*r2^2*t2_dot^2*cos(t2)*sin(t2) - g*l1*m2^2*r2^2*sin(t1 + t2)*cos(t2) + g*l1^2*m2^2*r2*cos(t2)*sin(t1) - g*m1*m2*r1^2*r2*sin(t1 + t2) + I1*l1*m2*r2*t1_dot^2*sin(t2) + I2*l1*m2*r2*t1_dot^2*sin(t2) + I2*l1*m2*r2*t2_dot^2*sin(t2) + g*m1*m2*r1*r2^2*sin(t1) + 2*l1^2*m2^2*r2^2*t1_dot*t2_dot*cos(t2)*sin(t2) + l1*m1*m2*r1^2*r2*t1_dot^2*sin(t2) + 2*I2*l1*m2*r2*t1_dot*t2_dot*sin(t2) + g*l1*m1*m2*r1*r2*cos(t2)*sin(t1))/(- l1^2*m2^2*r2^2*cos(t2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dz(3) = t1_ddot;
dz(4) = t2_ddot;

Y = [t1_ddot, cos(t2)*(2*t1_ddot + t2_ddot) - 2*sin(t2)*t1_dot*t2_dot - sin(t2)*t2_dot^2, t2_ddot, -sin(t1)*g, -sin(t1 + t2)*g;
    0, sin(t2)*t1_dot^2 + cos(t2)*t1_ddot, t1_ddot + t2_ddot, 0, -sin(t1+t2)*g];

M1 = [1,2*cos(t2),0,0,0];
M2 = [0,cos(t2),1,0,0];
M3 = [0,cos(t2),1,0,0];
M4 = [0,0,1,0,0];
M_hat = [[M1;M3]*alpha_hat,[M2;M4]*alpha_hat];

B = [0,0;0,0;1,0;0,1];
Phi = M_hat\Y;
alpha_dot = -Gamma\(Phi'*B'*P*x);

% t1_ddot = T(1);
% t2_ddot = T(2);
% v = -Kn*(e) + vd;
% u = M*v + C*q_dot + G;
% u1 = T(1);
% u2 = T(2);

dz(5) = alpha_dot(1);
dz(6) = alpha_dot(2);
dz(7) = alpha_dot(3);
dz(8) = alpha_dot(4);
dz(9) = alpha_dot(5);
dz(10) = T(1);
dz(11) = T(2);

end