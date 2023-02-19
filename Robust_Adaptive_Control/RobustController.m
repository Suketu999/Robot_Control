% syms M1 M2 M3 M4 C1 C2 C3 C4 g1 g2 real
% syms l1 l2 r1 r2 t1 t2 t1_dot t2_dot m1 m2 I1 I2 g u1 u2 t1_ddot t2_ddot real %X1 X2 X3 X4 %t stands for theta
% syms a b d a_hat b_hat d_hat real
% syms m1_hat m2_hat I1_hat I2_hat real
% 
% q = [t1;t2];
% q_dot = [t1_dot;t2_dot];
% q_ddot = [t1_ddot; t2_ddot];
% q_des = [t1_des;t2_des];
% q_dot_des = [t1_dot_des;t2_dot_des];
% q_ddot_des = [t1_ddot_des;t2_ddot_des];
% e = q - q_des;
% e_dot = q_dot - q_dot_des;
% e_ddot = q_ddot - q_ddot_des;
% x = [e;e_dot];
% x_dot = [e_dot;e_ddot];
% % x_des = [q_des;q_dot_des];
% 
% m1_hat = 0.75;
% m2_hat = 0.75;
% I1_hat = 0.063;
% I2_hat = 0.063;
% m1=1;
% m2=1;
% l1=1;
% l2=1;
% r1=0.45;
% r2=0.45;
% I1=0.084;
% I2=0.084;
% g=9.81;
% 
% M = [a+2*b*cos(t2), d+b*cos(t2); d+b*cos(t2), d];
% C = [-b*sin(t2)*t2_dot, -b*sin(t2)*(t1_dot+t2_dot); b*sin(t2)*t1_dot,0];
% G = [-m1*g*r1*sin(t1)-m2*g*(l1*sin(t1)+r2*sin(t1+t2)); -m2*g*r2*sin(t1+t2)];
% % where
% a = I1 + I2 + m1*r1^2 + m2*(l1^2 + r2^2);
% b = m2*l1*r2;
% d = I2 + m2*r2^2;
% % T = M*q_ddot + C*q_dot + G;
% 
% M_hat= [a_hat+2*b_hat*cos(t2), d_hat+b_hat*cos(t2); d_hat+b_hat*cos(t2), d_hat];
% C_hat= [-b_hat*sin(t2)*t2_dot, -b_hat*sin(t2)*(t1_dot+t2_dot); b_hat*sin(t2)*t1_dot,0];
% G_hat= [-m1_hat*g*r1*sin(t1)-m2_hat*g*(l1*sin(t1)+r2*sin(t1+t2)); -m2_hat*g*r2*sin(t1+t2)];
% % where
% a_hat = I1_hat + I2_hat + m1_hat*r1^2 + m2_hat*(l1^2 + r2^2);
% b_hat = m2_hat*l1*r2;
% d_hat = I2_hat + m2_hat*r2^2;
% % T = M_hat*q_ddot + C_hat*q_dot + G_hat;

A = [0,0,1,0;0,0,0,1;0,0,0,0;0,0,0,0];
B = [0,0;0,0;1,0;0,1];
lambda = [-3;-3;-4;-4];
Kn = place(A,B,lambda);
kp = Kn(:,1:2);
kd = Kn(:,3:4);

tspan = [0 10];
q0 = [200;125;0;0;0;0]*pi/180;
% phi = 0;
phi = 0.035;

Acl = [0,0,1,0;0,0,0,1;-kp(1,:),-kd(1,:);-kp(2,:),-kd(2,:)];
Q = eye(4).*11;
P = lyap(Acl',Q);

% v = q_ddot_des - kp*e - kd*e_dot + vr;
% e_ddot = -kp*e - kd*e_dot + vr + n;

[T,X] = ode45(@(t,x)ode_robust(x,t,kp,kd,P,phi),tspan,q0);

% T = M*v + C*q_dot + G;
plot(T,X(:,1),'LineWidth',2);
hold on
plot(t,t1_des,'lineWidth',2.0);
legend('Actual','Desired');
title('Theta1 vs. Time');
xlabel('t(sec)') 
ylabel('Theta 1') 

figure
plot(T,X(:,2),'LineWidth',2);
hold on
plot(t,t2_des,'lineWidth',2.0);
legend('Actual','Desired');
title('Theta2 vs. Time');
xlabel('t(sec)') 
ylabel('Theta 2') 

figure
plot(T,X(:,3),'LineWidth',2);
hold on
plot(t,t1_dot_des,'lineWidth',2.0);
legend('Actual','Desired');
title('Theta1Dot vs. Time');
xlabel('t(sec)') 
ylabel('Theta 1 dot')

figure
plot(T,X(:,4),'LineWidth',2);
hold on
plot(t,t2_dot_des,'lineWidth',2.0);
legend('Actual','Desired');
title('Theta2Dot vs. Time');
xlabel('t(sec)') 
ylabel('Theta 2 dot')

f=2;
U_1 = 0;
U_2 = 0;
Xsize =size(X);
while f <= Xsize(1)
%     U_1(f,1) = - (1189312871105339*X(f,1))/35184372088832 - (7292630729812249*X(f,2))/1125899906842624 - (7419*X(f,3))/500 - (4419*X(f,4))/1000;
%     U_2(f,1) = - (2430265662578031*X(f,1))/281474976710656 - (6233124955178157*X(f,2))/1125899906842624 - (4419*X(f,3))/1000 - (1719*X(f,4))/1000;
%     U = M_hat*v + C_hat*X(f,3:4) + G_hat;
    U_1(f) = (X(f,5)-X(f-1,5))/(T(f)-T(f-1));
    U_2(f) = (X(f,6)-X(f-1,6))/(T(f)-T(f-1));
    f = f+1;
end

figure(5);
plot (T,U_1,'lineWidth',2.0);
title('U1 v/s. Time')
xlabel('t(sec)') 
ylabel('U1 (units)')

figure(6);
plot (T,U_2,'lineWidth',2.0);
title('U2 v/s. Time')
xlabel('t(sec)') 
ylabel('U2 (units)')