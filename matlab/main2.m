%%%%%%%%%%%%%%%%%%%%% General
clc;
clear; close all
start=clock;  % start time
step=0.01;
time=10;%20
tnum=time/step+1;% set time
% tnum=1
%%%%%%%%%%%%%%%%%%%%%%%

l1=0.425;   
l2=0.39225;
l3=0.09475;
l4=0.0;

%%%%%%%%%%%%%%%%%%%%%%% parameter initialize, 2-D space
v=[17.53 38.60 74.39+90 180 0 0 0 0]'*pi/180;%4-link
% v=[0 0 0 0 0 0 0 0]';%4-link
x=tran(v,l1,l2,l3,l4);
x0(1)=-0.552
x0(2)=0.481

omega=0.5;%0.5
bq2=pi/20;%0.12
bq3=pi/20;
radiusx=0.1;

Kp=10;
Kpx=10;

kq2=10;%=200 activate =0 no use
kq3=10;
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
    
    fq2=q(2)^2-bq2^2;
    fq3=q(3)^2-bq3^2;
    
    regq2=kq2*min(0,fq2);
    regq3=kq3*min(0,fq3);
    
    regq=[0 regq2 regq3 0]';
    recregq(:,i)=regq;

    dx=jac(v,l1,l2,l3,l4);
    recdx(:,i)=dx;
    %%%%
    t=step*(i-1);
    
    qd=[q(1) q(2) q(3) q(4)]';
    dqd=[0 0 0 0]';
    
    errq=q-qd;
    recerrq(:,i)=errq;    

    xd=[x0(1)+radiusx*cos(omega*t)  x0(2)+radiusx*sin(omega*t)]';%[0.2+0.1*cos(omega*t) 0.4+0.1*sin(omega*t)]';
    %[a+r1*cos(omega*t)/(1+sin(omega*t)^2) b+r2*cos(omega*t)*sin(omega*t)/(1+sin(omega*t)^2)]';
    dxd=[-radiusx*omega*sin(omega*t) radiusx*omega*cos(omega*t)]';%[-0.1*omega*sin(omega*t) 0.1*omega*cos(omega*t)]';
    %[(-r1*omega*sin(omega*t)*(1+sin(omega*t)^2)-r1*omega*cos(omega*t)^2*2*sin(omega*t))/(1+sin(omega*t)^2)^2 ((-r2*omega*sin(omega*t)^2+r2*omega*cos(omega*t)^2)*(1+sin(omega*t)^2)-r2*omega*sin(omega*t)^2*cos(omega*t)^2*2)/(1+sin(omega*t)^2)^2]';
    
    recxd(:,i)=xd;
    recerrx(:,i)=x-xd;
        
    J=Jh(v,l1,l2,l3,l4)
    pJ=J'*inv(J*J')
    
    N=eye(4)-pJ*J;
    Nmatrix=null(J);
    
    %u=pJ*(dxd-Kpx*(x-xd))+N*(dqd-Kp*(q-qd));
    u=pJ*(dxd-Kpx*(x-xd))+N*regq;
    
    recdet(:,i)=det(J*J');
    
       
    speed=u;
    
    if i==1
        angle=q;
    else
        angle=angle+step*speed;
    end
    for i=1:1:4
        if angle(i)>2*pi
            angle(i)=2*pi
        else
            if angle(i)<-2*pi
                angle(i)=-2*pi
            end
        end
    end
    v=[angle 
       speed];
    
end

tt=0:step:time;

figure;
subplot(2,2,1);
plot(recx(1,:),recx(2,:),'b',recxd(1,:),recxd(2,:),'r:');
% xlabel('time (s)', 'FontSize', 20);ylabel('Force (N)', 'FontSize', 20);
% legend('f_{e1}', 'f_{e2}', 'weight', 4);
% axis([-2 2 -2 2]);
axis equal;
grid on;

subplot(2,2,2);
plot(tt, recerrx);
% xlabel('time (s)', 'FontSize', 20);ylabel('Force (N)', 'FontSize', 20);
% legend('f_{e1}', 'f_{e2}', 'weight', 4);
% axis([0 20 -1 1]);
grid on;

subplot(2,2,3);
plot(tt, recq);
% xlabel('time (s)', 'FontSize', 20);ylabel('Force (N)', 'FontSize', 20);
legend('q1', 'q2', 'q3', 'q4');
% axis([0 20 -10*pi 10*pi]);
grid on;

subplot(2,2,4);
plot(tt, recregq);
% xlabel('time (s)', 'FontSize', 20);ylabel('Force (N)', 'FontSize', 20);
legend('q1', 'q2', 'q3', 'q4');
% axis([0 20 -10 10]);
grid on;

figure;
subplot(2,1,1)
plot(tt,recxd(1,:),'r')
hold on;
plot(tt,recxd(2,:),'b')
hold off
subplot(2,1,2)
plot(tt, recdet);
% xlabel('time (s)', 'FontSize', 20);ylabel('Force (N)', 'FontSize', 20);
% legend('f_{e1}', 'f_{e2}', 'weight', 4);
% axis([0 20 -0.1 0.1]);
grid on;






