%risi.m

load fobs.dat
x=fobs;

i=x(:,1);

ua=x(:,2);
ub=x(:,3);

ia=x(:,4);
ib=x(:,5);

flxa=x(:,6);
flxb=x(:,7);

pll_ang=x(:,8);
pll_we=x(:,9);


figure(1)
plot(i, ua, i, ub);
grid on
legend('ua','ub');

figure(2)
plot(i, ia, i, ib);
grid on
legend('ia','ib');

figure(3)
plot(i, flxa, i, flxb);
grid on
legend('flxa','flxb');

figure(4)
subplot(211)
plot(i, pll_ang);
grid on
legend('pll ang');

subplot(212)
plot(i, pll_we);
grid on
legend('pll we');
