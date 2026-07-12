% main02_3DOF:
clear all; close all; clc;
S0 = 1; dS = 1;
t1 = 3; t2 = 6; t3 = 9;
i = 1;
for t = 0:0.1:t3
    res = Sshape_3DOF(t,S0,dS,t1,t2,t3);
    time(i) = t;
    p(i) = res.pos;
    v(i) = res.vel;
    a(i) = res.acc;
    i = i+1;
end

figure;
subplot(3,1,1); plot(time,p); grid
subplot(3,1,2); plot(time,v); grid
subplot(3,1,3); plot(time,a); grid