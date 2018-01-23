%risi.m

load pmsm.dat

time=pmsm(:,1);
isd=pmsm(:,2);
isq=pmsm(:,3);
usd=pmsm(:,4);
usq=pmsm(:,5);
isa=pmsm(:,6);
isb=pmsm(:,7);
usa=pmsm(:,8);
usb=pmsm(:,9);
torque=pmsm(:,10);
we=pmsm(:,11);
mech_angle=pmsm(:,12);
electrical_angle=pmsm(:,13);
wm=pmsm(:,14);



figure(1)
plot(time,isd,time,isq);
grid on
legend('isd','isq');

figure(2)
plot(time,usd,time,usq);
grid on
legend('usd','usq');

figure(3)
plot(time,isa,time,isb);
grid on
legend('isa','isb');

figure(4)
plot(time,usa,time,usb);
grid on
legend('usa','usb');

figure(5)
plot(time,torque);
grid on
legend('torque');

figure(6)
plot(time,mech_angle);
grid on
legend('mech_angle');

figure(7)
plot(time,wm);
grid on
legend('wm');

