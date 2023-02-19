syms l1 l2 r1 r2 t1 t2 t1_dot t2_dot m1 m2 I1 I2 g u1 u2 t1_ddot t2_ddot real

eqn1 = t2_ddot*(I2 + (m2*(2*r2^2 + 2*l1*cos(t2)*r2))/2) - g*(m2*(r2*sin(t1 + t2) + l1*sin(t1)) + m1*r1*sin(t1)) - u1 + t1_ddot*(m1*r1^2 + I1 + I2 + (m2*(2*l1^2 + 4*cos(t2)*l1*r2 + 2*r2^2))/2) - (m2*t2_dot*(2*l1*r2*sin(t2)*(t1_dot + t2_dot) + 2*l1*r2*t1_dot*sin(t2)))/2 == 0;
eqn2 = t2_ddot*(m2*r2^2 + I2) - u2 + t1_ddot*(I2 + (m2*(2*r2^2 + 2*l1*cos(t2)*r2))/2) - g*m2*r2*sin(t1 + t2) + l1*m2*r2*t1_dot*sin(t2)*(t1_dot + t2_dot) - l1*m2*r2*t1_dot*t2_dot*sin(t2) == 0;

%% Part a

eqn1_star = subs(eqn1,{t1_dot,t2_dot,t1_ddot,t2_ddot,u1,u2},{0,0,0,0,0,0});
eqn2_star = subs(eqn2,{t1_dot,t2_dot,t1_ddot,t2_ddot,u1,u2},{0,0,0,0,0,0});

% eqn1 = subs(eqn1,{m1,m2,l1,l2,r1,r2,I1,I2,g},{1,1,1,1,0.45,0.45,0.084,0.084,9.81})
% eqn2 = subs(eqn2,{m1,m2,l1,l2,r1,r2,I1,I2,g},{1,1,1,1,0.45,0.45,0.084,0.084,9.81})

% eqn1 = subs(eqn1,{t1,t2},{pi,pi})
% eqn2 = subs(eqn2,{t1,t2},{pi,pi})

[t1_star,t2_star] = solve([eqn1_star,eqn2_star],[t1,t2]);

assume(t1>0);
assume(t2>0);
[t1_star(end+1,1),t2_star(end+1,1)] = solve([eqn1_star,eqn2_star],[t1,t2])


%% Part b

q = [t1; t2];

q_dot = [t1_dot; t2_dot];

q_ddot = [t1_ddot; t2_ddot];

u = [u1; u2];

X_dot =[
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      t1_dot;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      t2_dot;
                                                                                                                                                                                                                                                                                                                                                                                                                      (u1*conj(I2) - u2*conj(I2) + u1*conj(m2)*conj(r2)^2 - u2*conj(m2)*conj(r2)^2 + sin(conj(t2))*conj(l1)*conj(m2)^2*conj(r2)^3*conj(t1_dot)^2 - u2*cos(conj(t2))*conj(l1)*conj(m2)*conj(r2) + sin(conj(t1))*conj(g)*conj(l1)*conj(m2)^2*conj(r2)^2 + sin(conj(t1))*conj(I2)*conj(g)*conj(l1)*conj(m2) + sin(conj(t1))*conj(I2)*conj(g)*conj(m1)*conj(r1) + sin(conj(t2))*conj(I2)*conj(l1)*conj(m2)*conj(r2)*conj(t1_dot)^2 - cos(conj(t2))*sin(conj(t1) + conj(t2))*conj(g)*conj(l1)*conj(m2)^2*conj(r2)^2 + sin(conj(t1))*conj(g)*conj(m1)*conj(m2)*conj(r1)*conj(r2)^2 + t2_dot*sin(conj(t2))*conj(l1)*conj(m2)^2*conj(r2)^3*conj(t1_dot) + t2_dot*sin(conj(t2))*conj(l1)*conj(m2)^2*conj(r2)^3*conj(t2_dot) + sin(conj(t2))*conj(l1)*conj(m2)^2*conj(r2)^3*conj(t1_dot)*conj(t2_dot) + cos(conj(t2))*sin(conj(t2))*conj(l1)^2*conj(m2)^2*conj(r2)^2*conj(t1_dot)^2 + t2_dot*sin(conj(t2))*conj(I2)*conj(l1)*conj(m2)*conj(r2)*conj(t1_dot) + t2_dot*sin(conj(t2))*conj(I2)*conj(l1)*conj(m2)*conj(r2)*conj(t2_dot) + cos(conj(t2))*sin(conj(t2))*conj(l1)^2*conj(m2)^2*conj(r2)^2*conj(t1_dot)*conj(t2_dot) + sin(conj(t2))*conj(I2)*conj(l1)*conj(m2)*conj(r2)*conj(t1_dot)*conj(t2_dot) - t2_dot*cos(conj(t2))*sin(conj(t2))*conj(l1)^2*conj(m2)^2*conj(r2)^2*conj(t1_dot))/(- cos(conj(t2))^2*conj(l1)^2*conj(m2)^2*conj(r2)^2 + conj(l1)^2*conj(m2)^2*conj(r2)^2 + conj(I2)*conj(l1)^2*conj(m2) + conj(m1)*conj(m2)*conj(r1)^2*conj(r2)^2 + conj(I1)*conj(m2)*conj(r2)^2 + conj(I2)*conj(m1)*conj(r1)^2 + conj(I1)*conj(I2));
-(u1*conj(I2) - u2*conj(I1) - u2*conj(I2) - u2*conj(l1)^2*conj(m2) - u2*conj(m1)*conj(r1)^2 + u1*conj(m2)*conj(r2)^2 - u2*conj(m2)*conj(r2)^2 + sin(conj(t2))*conj(l1)*conj(m2)^2*conj(r2)^3*conj(t1_dot)^2 + sin(conj(t2))*conj(l1)^3*conj(m2)^2*conj(r2)*conj(t1_dot)^2 + u1*cos(conj(t2))*conj(l1)*conj(m2)*conj(r2) - 2*u2*cos(conj(t2))*conj(l1)*conj(m2)*conj(r2) + sin(conj(t1))*conj(g)*conj(l1)*conj(m2)^2*conj(r2)^2 + sin(conj(t1))*conj(I2)*conj(g)*conj(l1)*conj(m2) + sin(conj(t1))*conj(I2)*conj(g)*conj(m1)*conj(r1) - sin(conj(t1) + conj(t2))*conj(g)*conj(l1)^2*conj(m2)^2*conj(r2) - sin(conj(t1) + conj(t2))*conj(I1)*conj(g)*conj(m2)*conj(r2) + sin(conj(t2))*conj(I1)*conj(l1)*conj(m2)*conj(r2)*conj(t1_dot)^2 + sin(conj(t2))*conj(I2)*conj(l1)*conj(m2)*conj(r2)*conj(t1_dot)^2 - cos(conj(t2))*sin(conj(t1) + conj(t2))*conj(g)*conj(l1)*conj(m2)^2*conj(r2)^2 + sin(conj(t1))*conj(g)*conj(m1)*conj(m2)*conj(r1)*conj(r2)^2 + t2_dot*sin(conj(t2))*conj(l1)*conj(m2)^2*conj(r2)^3*conj(t1_dot) - t2_dot*sin(conj(t2))*conj(l1)^3*conj(m2)^2*conj(r2)*conj(t1_dot) + t2_dot*sin(conj(t2))*conj(l1)*conj(m2)^2*conj(r2)^3*conj(t2_dot) - sin(conj(t1) + conj(t2))*conj(g)*conj(m1)*conj(m2)*conj(r1)^2*conj(r2) + sin(conj(t2))*conj(l1)*conj(m2)^2*conj(r2)^3*conj(t1_dot)*conj(t2_dot) + sin(conj(t2))*conj(l1)^3*conj(m2)^2*conj(r2)*conj(t1_dot)*conj(t2_dot) + 2*cos(conj(t2))*sin(conj(t2))*conj(l1)^2*conj(m2)^2*conj(r2)^2*conj(t1_dot)^2 + cos(conj(t2))*sin(conj(t1))*conj(g)*conj(l1)^2*conj(m2)^2*conj(r2) - t2_dot*sin(conj(t2))*conj(I1)*conj(l1)*conj(m2)*conj(r2)*conj(t1_dot) + t2_dot*sin(conj(t2))*conj(I2)*conj(l1)*conj(m2)*conj(r2)*conj(t1_dot) + t2_dot*sin(conj(t2))*conj(I2)*conj(l1)*conj(m2)*conj(r2)*conj(t2_dot) + 2*cos(conj(t2))*sin(conj(t2))*conj(l1)^2*conj(m2)^2*conj(r2)^2*conj(t1_dot)*conj(t2_dot) + sin(conj(t2))*conj(l1)*conj(m1)*conj(m2)*conj(r1)^2*conj(r2)*conj(t1_dot)^2 + sin(conj(t2))*conj(I1)*conj(l1)*conj(m2)*conj(r2)*conj(t1_dot)*conj(t2_dot) + sin(conj(t2))*conj(I2)*conj(l1)*conj(m2)*conj(r2)*conj(t1_dot)*conj(t2_dot) + t2_dot*cos(conj(t2))*sin(conj(t2))*conj(l1)^2*conj(m2)^2*conj(r2)^2*conj(t2_dot) - t2_dot*sin(conj(t2))*conj(l1)*conj(m1)*conj(m2)*conj(r1)^2*conj(r2)*conj(t1_dot) + cos(conj(t2))*sin(conj(t1))*conj(g)*conj(l1)*conj(m1)*conj(m2)*conj(r1)*conj(r2) + sin(conj(t2))*conj(l1)*conj(m1)*conj(m2)*conj(r1)^2*conj(r2)*conj(t1_dot)*conj(t2_dot))/(- cos(conj(t2))^2*conj(l1)^2*conj(m2)^2*conj(r2)^2 + conj(l1)^2*conj(m2)^2*conj(r2)^2 + conj(I2)*conj(l1)^2*conj(m2) + conj(m1)*conj(m2)*conj(r1)^2*conj(r2)^2 + conj(I1)*conj(m2)*conj(r2)^2 + conj(I2)*conj(m1)*conj(r1)^2 + conj(I1)*conj(I2))];

X_dot = subs(X_dot,{m1,m2,l1,l2,r1,r2,I1,I2,g},{1,1,1,1,0.45,0.45,0.084,0.084,9.81})

[A] = jacobian (X_dot,[q;q_dot])

[B] = jacobian (X_dot,u)

%% Part c

subA = subs(A,{t1,t2,t1_dot,t2_dot,t1_ddot,t2_ddot,u1,u2},{0,0,0,0,0,0,0,0});
disp('Eigenvalues for equilibrium point: 0,0 are:');
[EV1] = [eig(subA)];
EV1 = double([EV1])
disp('System is unstable at this equilibrium point');

subA = subs(A,{t1,t2,t1_dot,t2_dot,t1_ddot,t2_ddot,u1,u2},{pi,0,0,0,0,0,0,0});
disp('Eigenvalues for equilibrium point: pi,0 are:');
[EV2] = [eig(subA)];
EV2 = double([EV2])
disp('System is marginally stable at this equilibrium point');

subA = subs(A,{t1,t2,t1_dot,t2_dot,t1_ddot,t2_ddot,u1,u2},{0,pi,0,0,0,0,0,0});
disp('Eigenvalues for equilibrium point: 0,pi are:');
[EV3] = [eig(subA)];
EV3 = double([EV1])
disp('System is unstable at this equilibrium point');

subA = subs(A,{t1,t2,t1_dot,t2_dot,t1_ddot,t2_ddot,u1,u2},{pi,pi,0,0,0,0,0,0});
disp('Eigenvalues for equilibrium point: pi,pi are:');
[EV4] = [eig(subA)];
EV4 = double([EV1])
disp('System is unstable at this equilibrium point');

% pos = 1;
% 
% for i=1:3
%     subA = subs(A,{t1,t2,t1_dot,t2_dot,t1_ddot,t2_ddot,u1,u2},{t1_star(i),t2_star(i),0,0,0,0,0,0});
%     disp('Eigenvalues for equilibrium point:');
%     disp(t1_star(i),t2_star(i));
%     [EV(i,:)]=[eig(subA)]'
%     
%     for j=1:4
%         if EV(i,j) > 0
%             pos = pos*0;
%             break;
%         elseif EV(i,j) == 0
%             pos = pos*(-1);
%         end
%         pos = 1;
%     end
%     
%     if pos==0
%         disp('Unstable at:');
%         disp(t1_star(i),t2_star(i));
%     elseif pos==-1
%         disp('Asymptotically Stable at:');
%         disp(t1_star(i),t2_star(i));
%     else
%         disp('Stable at:');
%         disp(t1_star(i),t2_star(i));
%     end
%     
% end

%% Part d

uA = subs(A,{t1,t2,t1_dot,t2_dot,t1_ddot,t2_ddot,u1,u2},{0,0,0,0,0,0,0,0});

uB = subs(B,{t1,t2,t1_dot,t2_dot,t1_ddot,t2_ddot,u1,u2},{0,0,0,0,0,0,0,0});

Co = ctrb(uA,uB);

Rank = rank(Co)

disp('Since Controllability matrix is full rank, system is Controllable at 0,0');

%% Part e

lambda = [-1;-5;-3-1j;-3+1j];
A = double(uA);
B = double(uB);
disp('State-Feedback control is given by u = -kx. The value of k is as follows:');
Kn = place(A,B,lambda)
u = -Kn*[q;q_dot]

%% Part f

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

f=1;
U_1 = 0;
U_2 = 0;
zsize =size(z);
while f <= zsize(1)
    U_1(f,1) = - (1189312871105339*z(f,1))/35184372088832 - (7292630729812249*z(f,2))/1125899906842624 - (7419*z(f,3))/500 - (4419*z(f,4))/1000;
    U_2(f,1) = - (2430265662578031*z(f,1))/281474976710656 - (6233124955178157*z(f,2))/1125899906842624 - (4419*z(f,3))/1000 - (1719*z(f,4))/1000;
    f = f+1;
end

figure(5);
plot (t,U_1);
title('U1 v/s. Time')
xlabel('t(sec)') 
ylabel('U1 (units)')

figure(6);
plot (t,U_2);
title('U2 v/s. Time')
xlabel('t(sec)') 
ylabel('U2 (units)')