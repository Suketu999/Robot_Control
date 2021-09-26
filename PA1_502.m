clear
syms l1 l2 r1 r2 t1 t2 t1_dot t2_dot m1 m2 I1 I2 g u1 u2 t1_ddot t2_ddot %X1 X2 X3 X4 %t stands for theta
%%sym v1 = r1*t1_dot
%%sym v2 = 

q = [t1; t2];

q_dot = [t1_dot; t2_dot];

q_ddot = [t1_ddot; t2_ddot];

u = [u1; u2];

K = (1/2)*m1*(r1^2)*(t1_dot^2) + (1/2)*m2*(l1^2*(t1_dot^2) + (r2^2)*(t1_dot + t2_dot)^2 + ...
            2*l1*r2*t1_dot*(t1_dot + t2_dot)*cos(t2)) + (1/2)*I1*(t1_dot^2) + ...
                (1/2)*I2*((t1_dot + t2_dot)^2);
P = g*(m1*(r1*cos(t1)) + m2*(l1*cos(t1) + r2*cos(t2+t1)));

L = K - P;

dL_dqd = (jacobian (L, q_dot))'; %del L by del q_dot

ddL_dqddt = jacobian (dL_dqd, [q; q_dot])*[q_dot;q_ddot]; % d(del L by del q_dot) by dt

%ddL_dqddt = jacobian (dL_dqd, [q_dot])*[q_ddot];

%ddL_dqddt = jacobian (dL_dqd, [q])*[q_dot] + jacobian (dL_dqd, [q_dot])*[q_ddot]

dL_dq = (jacobian (L, q))'; %del L by del q

eqn1 = ddL_dqddt - dL_dq - u

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

X = [q; q_dot];
%eqn2 = X == [X1; X2; X3; X4];

sol = solve([eqn1==0],[t1_ddot,t2_ddot]);
t1_ddot = sol.t1_ddot
t2_ddot = sol.t2_ddot
X_dot = [q_dot;t1_ddot;t2_ddot]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[t,z] = ode45(@ode_progassn, [0,10], (pi/180*[30; 45; 0; 0]));

figure(1);
plot (t,rad2deg(z(:,1)));
title('Theta 1 v/s. Time')
xlabel('t(sec)') 
ylabel('Theta 1 (degree)') 
figure(2);
plot (t,rad2deg(z(:,2)));
title('Theta 2 v/s. Time')
xlabel('t(sec)') 
ylabel('Theta 2 (degree)') 
figure(3);
plot (t,rad2deg(z(:,3)));
title('Theta 1 dot v/s. Time')
xlabel('t(sec)') 
ylabel('Theta 1 dot (degree/s)')
figure(4);
plot (t,rad2deg(z(:,4)));
title('Theta 2 dot v/s. Time')
xlabel('t(sec)') 
ylabel('Theta 2 dot (degree/s)')

% % % tspan = [0 10];
% % % X0 = (pi/180)*[30; 45; 0; 0];
% % % [t, X] = ode45(@(t,X) X, tspan, X0);
% % % plot (t,X);