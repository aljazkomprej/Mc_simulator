%risi.m

clear all

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
idiv=1/10;
ty=y(:,1);

sector=y(:,2);
m1=y(:,3);
m2=y(:,4);
mudc=y(:,5);

t1up  =y(:,6);
t1down=y(:,7);
t2up  =y(:,8);
t2down=y(:,9);
t3up  =y(:,10);
t3down=y(:,11);

si1=y(:,12)*idiv;
si2=y(:,13)*idiv;
si3=y(:,14)*idiv;

ang=y(:,15);
duty_re=y(:,16);
duty_im=y(:,17);


tim=y(:,18);
ang1=y(:,19);
amp=y(:,20);

%%
figure(1)
%//plot(time, i1, time, i2, time, i3, time, iavg1, time, iavg2, time, iavg3, ty, si1, 'o-', ty, si2, 'o-', ty, si3, 'o-' );
%//grid on
%//legend('i1','i2','i3','iavg1','iavg2','iavg3','si1','si2','si3');
a(1)=subplot(2,1,1),
plot(ty, m1, 'k:x', ty, m2, 'm:x', ty, t1up, 'b', ty, t1down,'b', ty, t2up, 'r', ty, t2down, 'r', ty, t3up, 'g', ty, t3down,'g');
grid on
legend('m1','m2','t1up','t1down','t2up','t2down','t3up','t3down');
a(2)=subplot(2,1,2),
plot(time, i1, time, i2, time, i3, ty, si1, 'o-', ty, si2, 'o-', ty, si3, 'o-' );
grid on
legend('i1','i2','i3','si1','si2','si3');
linkaxes(a,'x')

figure(10)
subplot(1,3,1)
plot(tim, ang,'.-',tim,ang1,'.-'), grid on, ylabel('angle');
subplot(1,3,2)
plot(tim, amp,'.-'), grid on, ylabel('amplitude');
subplot(1,3,3)
plot(ty, duty_re,'.-', ty, duty_im ,'.-');
grid on
legend('duty re','duty im');

figure(7)
subplot(2,1,1),plot(ty, sector), grid on, ylabel('sector')
subplot(2,1,2),
plot(ty, m1, 'k:x', ty, m2, 'm:x', ty, t1up, 'b', ty, t1down,'b', ty, t2up, 'r', ty, t2down, 'r', ty, t3up, 'g', ty, t3down,'g');
grid on
legend('m1','m2','t1up','t1down','t2up','t2down','t3up','t3down');

figure(2)
plot(time, u1, time, u2, time, u3);
grid on
legend('u1','u2','u3');


%%
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
plot(time, iavg1, time, iavg2, time, iavg3, ty, si1, ty, si2, ty, si3);
grid on
legend('iavg1','iavg2','iavg3');



figure(8)
plot(time, i1, time, i2, time, i3 );
grid on
legend('i1','i2','i3');



