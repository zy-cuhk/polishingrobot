clc,clear,close all
mdl_ur5;
step=0.01;
time=50;
tnum=time/step+1;
omega=0.5;
radius=0.1;
kpr=10;
kq6=200;
bq6=2.2;
position_x=0.0;
position_y=0.0;
z_height=0.6;
q=[0,pi/2,pi/2,pi/2,0,0]';
T=ur5.fkine(q);
T1=[     1         0         0    2*radius;
         0         1         0       0.0;
         0         0         1   z_height;
         0         0         0         1];
q=ur5.ikine(T1)';
for i=1:1:tnum
    T1=ur5.fkine(q);
    r=transl(T1)';
    rpy_angle=T1.torpy('xyz');
%     r=[r1(1) r1(2) r1(3) rpy_angle(1) rpy_angle(2)]';
    
    t=step*(i-1);
    r_dsr=[radius*cos(omega*t)+position_x,radius*sin(omega*t)+position_y,z_height]';
    dr_dsr=[-radius*omega*sin(omega*t),radius*omega*cos(omega*t),0.0]';
    delta_r=(r-r_dsr);
    delta_rdot=dr_dsr-kpr*delta_r;
    
    jacob_mat=ur5.jacob0(q);
    J=jacob_mat(1:3,:);
    pJ=J'*inv(J*J');
    N=eye(6)-pJ*J;
%     u=pJ*delta_rdot;
    q(6)
    fq6=q(6)^2-bq6^2;
    regq6=kq6*min(0,fq6);
    regq=[0 0 0 0 0 regq6]';
    
    u=pJ*delta_rdot+N*regq;
    % u=pJ*delta_rdot;
    dq=u;
    q=q+dq*step;
    
    for j=1:1:6
        if q(j)>2*pi
            q(j)=2*pi;
        else if q(j)<-2*pi
                q(j)=-2*pi;
            end
        end
    end
    
    rec_r(:,i)=r;
    rec_r_dsr(:,i)=r_dsr;
    rec_err_r(:,i)=delta_r;
    rec_condition_num(:,i)=det(J*J');
    rec_regq(:,i)=regq;
    rec_q(:,i)=q;
    rec_rpy(:,i)=rpy_angle;
end

t=0:step:time;
figure;
subplot(2,2,1);
plot(t,rec_r);
grid on;
title("trans: r");

subplot(2,2,2);
plot(t,rec_r_dsr);
grid on;
title("trans: r_dsr");

subplot(2,2,3);
plot(t,rec_err_r);
grid on;
title("trans: error_r");

subplot(2,2,4);
plot(t,rec_condition_num);
grid on;
title("condition number")

figure;
subplot(3,1,1);
plot(t,rec_regq);
grid on;
title("the regq")
subplot(3,1,2);
plot(t,rec_q);
grid on;
title("the q")
subplot(3,1,3);
plot(t,rec_rpy);
grid on;
title("the rpy")

