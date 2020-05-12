clc,clear,close all
step=0.01;
time=20;
tnum=time/step+1;
mdl_ur5;
q=[0,pi/2,pi/2,pi/2,0,0]';
dq=[0,0,0,0,0,0]';


omega=5;
radius=pi/4;
Kp=50;
for i=1:1:tnum
    rec_q(:,i)=q;
    rec_dq(:,i)=dq;
    
    T1=ur5.fkine(q);
    xyz=transl(T1);
    rpy_angle=T1.torpy('xyz')
    rec_x(:,i)=xyz;
    rec_angle(:,i)=rpy_angle;
    
    t=step*(i-1);
    q_dsr=[0,pi/2+radius*sin(omega*t),-pi/2,pi/2,0,0]';
    dq_dsr=[0,radius*omega*cos(omega*t),0,0,0,0]';
    rec_q_dsr(:,i)=q_dsr;
    
    errq=q-q_dsr;
    rec_errq(i,:)=errq;
    dx_dsr=[0,0,0]';
    rec_dx_dsr(:,i)=dx_dsr;
    
    jacob_mat=ur5.jacob0(q);
    J=jacob_mat(1:3,:);
    pJ=J'*inv(J*J');
    N=eye(6)-pJ*J;
    u=pJ*dx_dsr+N*(dq_dsr-Kp*(q-q_dsr));
    dq=u;
    q=q+dq*step;

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



