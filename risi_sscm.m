%risi.m

load sscm.dat
x=sscm;

angfix=x(:,1);
ang=angfix/65536*360;

t1s=x(:,2);
t1e=x(:,3);

t2s=x(:,4);
t2e=x(:,5);

t3s=x(:,6);
t3e=x(:,7);

m1=x(:,8);
m2=x(:,9);
m3=x(:,10);
m4=x(:,11);
m_udc=x(:,12);


figure(1)
plot(ang, t1s, 'r', ang, t1e, 'r', ang, t2s, 'g', ang, t2e, 'g', ang, t3s, 'b', ang, t3e, 'b');
grid on
legend('t1s','t1e','t2s','t2e','t3s','t3e');


figure(2)
plot(ang, m1, ang, m2, ang, m3, ang, m4, ang, m_udc);
grid on
legend('m1','m2','m3','m4','mudc');

figure(3)
plot(ang, t1s, 'r', ang, t1e, 'r', ang, t2s, 'g', ang, t2e, 'g', ang, t3s, 'b', ang, t3e, 'b', ang, m1, ang, m2, ang, m3, ang, m4, ang, m_udc);
grid on
legend('t1s','t1e','t2s','t2e','t3s','t3e','m1','m2','m3','m4','mudc');

figure(4)
subplot(311)
plot(ang, t1s, ang, t1e);
grid on
legend('t1s','t1e');
subplot(312)
plot(ang, t2s, ang, t2e);
grid on
legend('t2s','t2e');
subplot(313)
plot(ang, t3s, ang, t3e);
grid on
legend('t3s','t3e');

