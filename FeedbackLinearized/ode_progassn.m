function dz = ode_progassn(t,z)
m1=1;m2=1;l1=1;l2=1;r1=0.45;r2=0.45;I1=0.084;I2=0.084;g=9.81;
dz=zeros(4,1);
z=num2cell(z);
[t1, t2, t1_dot, t2_dot] = deal(z{:});

if abs(t1)>2*pi
    t1 = mod(t1, 2*pi);
end

if abs(t2)>2*pi
    t2 = mod(t2, 2*pi);
end

x = [t1; t2; t1_dot; t2_dot];
q_dot = [t1_dot; t2_dot];

q_des = [(pi*t^3)/500 - (3*pi*t^2)/100 + pi;(pi*t^3)/1000 - (3*pi*t^2)/200 + pi/2];
q_dot_des = [(3*pi*t^2)/500 - (3*pi*t)/50;(3*pi*t^2)/1000 - (3*pi*t)/100];
x_des = [q_des;q_dot_des];
vd = [(3*pi*t)/250 - (3*pi)/50; (3*pi*t)/500 - (3*pi)/100];

M = [(m1*r1^2 + I1 + I2 + (m2*(2*l1^2 + 4*cos(t2)*l1*r2 + 2*r2^2))/2), (I2 + (m2*(2*r2^2 + 2*l1*cos(t2)*r2))/2);
    (I2 + (m2*(2*r2^2 + 2*l1*cos(t2)*r2))/2), (m2*r2^2 + I2)];
C = [- (m2*t2_dot*(2*l1*r2*sin(t2)+2*l1*r2*sin(t2)))/2, - (m2*t2_dot*(2*l1*r2*sin(t2)));
    l1*m2*r2*t1_dot*sin(t2) - l1*m2*r2*t2_dot*sin(t2), l1*m2*r2*t1_dot*sin(t2)];
G = [- g*(m2*(r2*sin(t1 + t2) + l1*sin(t1)) + m1*r1*sin(t1));
    - g*m2*r2*sin(t1 + t2)];

Kn = [26.3636   -0.9759   10.2714   -0.0273;
    0.8324   20.6796    1.1149    9.7286]

v = -Kn*(x - x_des) + vd;

u = M*v + C*q_dot + G;

u1 = u(1);
u2 = u(2);
dz(1) = t1_dot;
dz(2) = t2_dot;
dz(3) = (I2*u1 - I2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*r2^3*t1_dot^2*sin(t2) + l1*m2^2*r2^3*t2_dot^2*sin(t2) + g*l1*m2^2*r2^2*sin(t1) + I2*g*l1*m2*sin(t1) + I2*g*m1*r1*sin(t1) - l1*m2*r2*u2*cos(t2) + 2*l1*m2^2*r2^3*t1_dot*t2_dot*sin(t2) + l1^2*m2^2*r2^2*t1_dot^2*cos(t2)*sin(t2) - g*l1*m2^2*r2^2*sin(t1 + t2)*cos(t2) + I2*l1*m2*r2*t1_dot^2*sin(t2) + I2*l1*m2*r2*t2_dot^2*sin(t2) + g*m1*m2*r1*r2^2*sin(t1) + 2*I2*l1*m2*r2*t1_dot*t2_dot*sin(t2))/(- l1^2*m2^2*r2^2*cos(t2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dz(4) = -(I2*u1 - I1*u2 - I2*u2 - l1^2*m2*u2 - m1*r1^2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*r2^3*t1_dot^2*sin(t2) + l1^3*m2^2*r2*t1_dot^2*sin(t2) + l1*m2^2*r2^3*t2_dot^2*sin(t2) - g*l1^2*m2^2*r2*sin(t1 + t2) - I1*g*m2*r2*sin(t1 + t2) + g*l1*m2^2*r2^2*sin(t1) + I2*g*l1*m2*sin(t1) + I2*g*m1*r1*sin(t1) + l1*m2*r2*u1*cos(t2) - 2*l1*m2*r2*u2*cos(t2) + 2*l1*m2^2*r2^3*t1_dot*t2_dot*sin(t2) + 2*l1^2*m2^2*r2^2*t1_dot^2*cos(t2)*sin(t2) + l1^2*m2^2*r2^2*t2_dot^2*cos(t2)*sin(t2) - g*l1*m2^2*r2^2*sin(t1 + t2)*cos(t2) + g*l1^2*m2^2*r2*cos(t2)*sin(t1) - g*m1*m2*r1^2*r2*sin(t1 + t2) + I1*l1*m2*r2*t1_dot^2*sin(t2) + I2*l1*m2*r2*t1_dot^2*sin(t2) + I2*l1*m2*r2*t2_dot^2*sin(t2) + g*m1*m2*r1*r2^2*sin(t1) + 2*l1^2*m2^2*r2^2*t1_dot*t2_dot*cos(t2)*sin(t2) + l1*m1*m2*r1^2*r2*t1_dot^2*sin(t2) + 2*I2*l1*m2*r2*t1_dot*t2_dot*sin(t2) + g*l1*m1*m2*r1*r2*cos(t2)*sin(t1))/(- l1^2*m2^2*r2^2*cos(t2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);

end