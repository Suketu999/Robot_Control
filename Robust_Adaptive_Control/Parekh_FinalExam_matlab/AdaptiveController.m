% Y = [t1_ddot, cos(t2)*(2*t1_ddot + t2_ddot) - 2*sin(t2)*t1_dot*t2_dot - sin(t2)*t2_dot^2, t2_ddot, -sin(t1)*g, -sin(t1 + t2)*g;
%     0, sin(t2)*t1_dot^2 + cos(t2)*t1_ddot, t1_ddot + t2_ddot, 0, -sin(t1+t2)*g];
% I changed the variable names as per my notation in the above equation

m1=1;m2=1;l1=1;l2=1;r1=0.45;r2=0.45;I1=0.084;I2=0.084;g=9.81;
alpha = [m2*l1^2 + m1*r1^2 + m2*r2^2 + I1 + I2; m2*l1*r2; m2*r2^2 + I2; m1*r1 + m2*l1; m2*r2];
alpha_hat = 0.75*alpha;

A = [0,0,1,0;0,0,0,1;0,0,0,0;0,0,0,0];
B = [0,0;0,0;1,0;0,1];
lambda = [-3;-3;-4;-4];
Kn = place(A,B,lambda);
kp = Kn(:,1:2);
kd = Kn(:,3:4);

tspan = [0 10];
q0 = [deg2rad(200); deg2rad(125);0;0;alpha_hat(1);alpha_hat(2);alpha_hat(3);alpha_hat(4);alpha_hat(5);0;0];
Gamma = eye(5).*0.4;

Acl = [0,0,1,0;0,0,0,1;-kp(1,:),-kd(1,:);-kp(2,:),-kd(2,:)];
Q = eye(4)*5;
P = lyap(Acl',Q);
% P = eye(4)*0;

% v = q_ddot_des - kp*e - kd*e_dot + vr;
% e_ddot = -kp*e - kd*e_dot + vr + n;

[T,X] = ode45(@(t,z)ode_adapt(t,z,kp,kd,P,Gamma),tspan,q0);

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
    U_1(f) = (X(f,10)-X(f-1,10))/(T(f)-T(f-1));
    U_2(f) = (X(f,11)-X(f-1,11))/(T(f)-T(f-1));
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

%For comparision of Actual and Obtained Parameters
for i = 1:length(X)
    Z(i,1) = alpha(1) - X(i,5);
    Z(i,2) = alpha(2) - X(i,6);
    Z(i,3) = alpha(3) - X(i,7);
    Z(i,4) = alpha(4) - X(i,8);
    Z(i,5) = alpha(5) - X(i,9);
end

plot(T,Z(:,1:5),'linewidth',2)
title('Alpha-AlphaHat v/s. Time')
legend('Alpha(1)','Alpha(2)','Alpha(3)','Alpha(4)','Alpha(5)')