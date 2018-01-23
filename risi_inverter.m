%risi.m
tic;
clear all
clc

load inverter.dat
load inverter_slow.dat

x=inverter;
y=inverter_slow;

time=x(:,1);

i1=x(:,2);
i2=x(:,3);
i3=x(:,4);

u1=x(:,5);
u2=x(:,6);
u3=x(:,7);

e1=x(:,8);
e2=x(:,9);
e3=x(:,10);

mi1=x(:,11);
mi2=x(:,12);

uavg1=x(:,13);
uavg2=x(:,14);
uavg3=x(:,15);

iavg1=x(:,16);
iavg2=x(:,17);
iavg3=x(:,18);



uavg0=(uavg1+uavg2+uavg3)./3;
ua1=uavg1-uavg0;
ua2=uavg2-uavg0;
ua3=uavg3-uavg0;

% idiv=-90*47/49;
idiv=2^10/(8854*10); %2^10=šift v desno, 8853=ADC_gain, 10=KMEAS

ty=y(:,1);
ang=y(:,2);
mag=y(:,3);

u1y=y(:,4);
u2y=y(:,5);
u3y=y(:,6);



t1up  =y(:,7);
 t1down=y(:,8);
t2up  =y(:,9);
 t2down=y(:,10);
t3up  =y(:,11);
 t3down=y(:,12);

m1=y(:,13);
m2=y(:,14);

 si1=y(:,15)*idiv;
 si2=y(:,16)*idiv;
 si3=y(:,17)*idiv;

 mchp=y(:,18);

% duty_re=y(:,16);
% duty_im=y(:,17);
% 
% 
% tim=y(:,18);
% ang1=y(:,19);
% amp=y(:,20);


%%
figure(10)
% subplot(1,2,1)
% plot(ty, ang,'.-'), grid on, ylabel('angle'), title;
subplot(1,2,1)
plot(ty, u1y,'.-',ty, u2y,'.-',ty, u3y,'.-'), grid on, ylabel('amplitude'),
title('reference (input) voltage'),
legend('u1r','u2r','u3r');
subplot(1,2,2),
plot( ty, t1up, 'b*-', ty, t1down,'b.-', ty, t2up, 'r*-', ty, t2down, 'r.-', ty, t3up, 'g*-', ty, t3down,'g.-',ty, m1, 'k:x', ty, m2, 'm:x');
%plot(ty, t1up, 'b', ty, t2up, 'r', ty, t3up, 'g');
grid on
%legend('t1up','t2up','t3up');
legend('t1up','t1down','t2up','t2down','t3up','t3down','m1','m2');
 

figure(2)
plot(time, u1, time, u2, time, u3);
grid on
legend('u1','u2','u3');


figure(1)
%//plot(time, i1, time, i2, time, i3, time, iavg1, time, iavg2, time, iavg3, ty, si1, 'o-', ty, si2, 'o-', ty, si3, 'o-' );
%//grid on
%//legend('i1','i2','i3','iavg1','iavg2','iavg3','si1','si2','si3');
a(1)=subplot(2,1,1),
plot(ty, m1, 'k:x', ty, m2, 'm:x', ty, t1up, 'b*-', ty, t1down,'b.-', ty, t2up, 'r*-', ty, t2down, 'r.-', ty, t3up, 'g*-', ty, t3down,'g.-');
grid on
legend('m1','m2','t1up','t1down','t2up','t2down','t3up','t3down');
a(2)=subplot(2,1,2),
plot(time, i1, time, i2, time, i3, ty, si1, 'o-', ty, si2, 'o-', ty, si3, 'o-' );
grid on
legend('i1','i2','i3','si1','si2','si3');
linkaxes(a,'x')


figure(3)
plot(time, uavg1, time, uavg2, time, uavg3, time, ua1, time, ua2, time, ua3);
grid on
legend('uavg1','uavg2','uavg3','ua1','ua2','ua3');

figure(4)
plot(time, e1, time, e2, time, e3);
grid on
legend('e1','e2','e3');

figure(5)
plot(time, mi1, time, mi2);
grid on
legend('mi1','mi2');

figure(6)
plot(time, iavg1, time, iavg2, time, iavg3, ty, si1, ty, si2, ty, si3, ty, 10*mchp);
grid on
legend('iavg1','iavg2','iavg3','si1','si2','si3','hp');



% figure(8)
% plot(time, i1, time, i2, time, i3 );
% grid on
% legend('i1','i2','i3');

toc


