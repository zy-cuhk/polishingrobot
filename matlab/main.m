%%%%%%%%%%%%%%%%%%%%% General
clc;
clear;
start=clock;  % start time
step=0.004;
time=20;%20
tnum=time/step+1;% set time
%%%%%%%%%%%%%%%%%%%%%%%

l1=0.4;   
l2=0.4;
l3=0.4;
l4=0.4;

%%%%%%%%%%%%%%%%%%%%%%% parameter initialize, 2-D space
v=[pi/4 pi/3 pi/2 pi/2 0 0 0 0]';%4-link
x=tran(v,l1,l2,l3,l4);
omega=0.5;%0.5
radius=pi/8;%0.12

Kp=500;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
starttimer=0;
%%%%%%%%%%%%%%%%%%%%%%% Main Loop
for i=1:tnum
    q =[v(1) v(2) v(3) v(4)]';
    dq=[v(5) v(6) v(7) v(8)]';
    
    recq(:,i)=q;
    recdq(:,i)=dq;
    
    s1=sin(v(1)); c1=cos(v(1));
    s2=sin(v(2)); c2=cos(v(2));
    s12=sin(v(1)+v(2)); c12=cos(v(1)+v(2));
    s123=sin(v(1)+v(2)+v(3)); c123=cos(v(1)+v(2)+v(3));
    s1234=sin(v(1)+v(2)+v(3)); c1234=cos(v(1)+v(2)+v(3));
    
    x=tran(v,l1,l2,l3,l4); % true cartesian space position
    recx(:,i)=x;

    dx=jac(v,l1,l2,l3,l4);
    recdx(:,i)=dx;
    %%%%
    t=step*(i-1);
    
    qd=[q(1) q(2) pi/2+radius*sin(omega*t) q(4)]';
    dqd=[0 0 radius*omega*sin(omega*t) 0]';
    
    errq=q-qd;
    recerrq(:,i)=errq;    

    %xd=[0.2+radius*cos(omega*t) 0.4+radius*sin(omega*t)]';%[0.2+0.1*cos(omega*t) 0.4+0.1*sin(omega*t)]';
    %[a+r1*cos(omega*t)/(1+sin(omega*t)^2) b+r2*cos(omega*t)*sin(omega*t)/(1+sin(omega*t)^2)]';
    dxd=[0 0]';%[-0.1*omega*sin(omega*t) 0.1*omega*cos(omega*t)]';
    %[(-r1*omega*sin(omega*t)*(1+sin(omega*t)^2)-r1*omega*cos(omega*t)^2*2*sin(omega*t))/(1+sin(omega*t)^2)^2 ((-r2*omega*sin(omega*t)^2+r2*omega*cos(omega*t)^2)*(1+sin(omega*t)^2)-r2*omega*sin(omega*t)^2*cos(omega*t)^2*2)/(1+sin(omega*t)^2)^2]';
    
    recdxd(:,i)=dxd;
        
    J=Jh(v,l1,l2,l3,l4);
    pJ=J'*inv(J*J');
    
    N=eye(4)-pJ*J;
    Nmatrix=null(J);
    
    u=pJ*dxd+N*(dqd-Kp*(q-qd));
    
    speed=u;
    
    if i==1
        angle=q;
    else
        angle=angle+step*speed;
    end
    
    v=[angle 
       speed];
    
end

tt=0:step:time;

figure;
plot(tt, recx);
% xlabel('time (s)', 'FontSize', 20);ylabel('Force (N)', 'FontSize', 20);
% legend('f_{e1}', 'f_{e2}', 'weight', 4);
axis([0 20 -1 1]);
grid on;

figure;
plot(tt, recq);
% xlabel('time (s)', 'FontSize', 20);ylabel('Force (N)', 'FontSize', 20);
% legend('f_{e1}', 'f_{e2}', 'weight', 4);
axis([0 20 -5 5]);
grid on;