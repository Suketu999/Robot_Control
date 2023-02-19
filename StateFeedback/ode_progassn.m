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

u1 = - (1189312871105339*t1)/35184372088832 - (7292630729812249*t2)/1125899906842624 - (7419*t1_dot)/500 - (4419*t2_dot)/1000;
u2 = - (2430265662578031*t1)/281474976710656 - (6233124955178157*t2)/1125899906842624 - (4419*t1_dot)/1000 - (1719*t2_dot)/1000;
dz(1) = t1_dot;
dz(2) = t2_dot;
dz(3) = (I2*u1 - I2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*r2^3*t1_dot^2*sin(t2) + l1*m2^2*r2^3*t2_dot^2*sin(t2) + g*l1*m2^2*r2^2*sin(t1) + I2*g*l1*m2*sin(t1) + I2*g*m1*r1*sin(t1) - l1*m2*r2*u2*cos(t2) + 2*l1*m2^2*r2^3*t1_dot*t2_dot*sin(t2) + l1^2*m2^2*r2^2*t1_dot^2*cos(t2)*sin(t2) - g*l1*m2^2*r2^2*sin(t1 + t2)*cos(t2) + I2*l1*m2*r2*t1_dot^2*sin(t2) + I2*l1*m2*r2*t2_dot^2*sin(t2) + g*m1*m2*r1*r2^2*sin(t1) + 2*I2*l1*m2*r2*t1_dot*t2_dot*sin(t2))/(- l1^2*m2^2*r2^2*cos(t2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dz(4) = -(I2*u1 - I1*u2 - I2*u2 - l1^2*m2*u2 - m1*r1^2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + l1*m2^2*r2^3*t1_dot^2*sin(t2) + l1^3*m2^2*r2*t1_dot^2*sin(t2) + l1*m2^2*r2^3*t2_dot^2*sin(t2) - g*l1^2*m2^2*r2*sin(t1 + t2) - I1*g*m2*r2*sin(t1 + t2) + g*l1*m2^2*r2^2*sin(t1) + I2*g*l1*m2*sin(t1) + I2*g*m1*r1*sin(t1) + l1*m2*r2*u1*cos(t2) - 2*l1*m2*r2*u2*cos(t2) + 2*l1*m2^2*r2^3*t1_dot*t2_dot*sin(t2) + 2*l1^2*m2^2*r2^2*t1_dot^2*cos(t2)*sin(t2) + l1^2*m2^2*r2^2*t2_dot^2*cos(t2)*sin(t2) - g*l1*m2^2*r2^2*sin(t1 + t2)*cos(t2) + g*l1^2*m2^2*r2*cos(t2)*sin(t1) - g*m1*m2*r1^2*r2*sin(t1 + t2) + I1*l1*m2*r2*t1_dot^2*sin(t2) + I2*l1*m2*r2*t1_dot^2*sin(t2) + I2*l1*m2*r2*t2_dot^2*sin(t2) + g*m1*m2*r1*r2^2*sin(t1) + 2*l1^2*m2^2*r2^2*t1_dot*t2_dot*cos(t2)*sin(t2) + l1*m1*m2*r1^2*r2*t1_dot^2*sin(t2) + 2*I2*l1*m2*r2*t1_dot*t2_dot*sin(t2) + g*l1*m1*m2*r1*r2*cos(t2)*sin(t1))/(- l1^2*m2^2*r2^2*cos(t2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);

end