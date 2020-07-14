clc,clear,close all
mdl_ur5;
step=0.01;
time=50;
tnum=time/step+1;
omega=0.5;
radius=0.3;
Kp=50;
position_x=0.0;
position_y=0.0;
z_height=0.6;
q=[0,pi/2,pi/2,pi/2,0,0]';

for i=1:1:tnum
    T1=ur5.fkine(q);
    xyz=transl(T1);
    rpy_angle=T1.torpy('xyz');
    
    dx_dsr=[0,0,0]';
    t=step*(i-1);
    q_dsr=[0,pi/2+radius*sin(omega*t),-pi/2,pi/2,0,0]';
    dq_dsr=[0,radius*omega*cos(omega*t),0,0,0,0]';
    errq=q-q_dsr;
    
    jacob_mat=ur5.jacob0(q);
    J=jacob_mat(1:3,:);
    pJ=J'*inv(J*J');
    N=eye(6)-pJ*J;
    u=pJ*dx_dsr+N*(dq_dsr-Kp*(q-q_dsr));
    dq=u;
    q=q+dq*step;
    
    rec_q(:,i)=q;
    rec_q_dsr(:,i)=q_dsr;
    rec_errq(i,:)=errq;
    rec_dx_dsr(:,i)=dx_dsr;
    rec_dq(:,i)=dq;
    rec_x(:,i)=xyz;
    rec_angle(:,i)=rpy_angle;
    
end

t=0:step:time;
figure;
subplot(2,2,1);
plot(t,rec_x);
grid on;
title("trans: xyz");

subplot(2,2,2);
plot(t,rec_q);
grid on;
% hold on;
% plot(t,rec_q_dsr);
title("joint angles");

subplot(2,2,3);
plot(t,rec_errq);
grid on;
title("joint angle errors");

subplot(2,2,4);
plot(t,rec_angle);
grid on;
title("rot: rpy")



