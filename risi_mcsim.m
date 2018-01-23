%risi.m

load mcwopwm.dat
x=mcwopwm;

i=x(:,1);

ua=x(:,2);
ub=x(:,3);

ud=x(:,4);
uq=x(:,5);

ud_duty=x(:,6);
uq_duty=x(:,7);

ia=x(:,8);
ib=x(:,9);

id=x(:,10);
iq=x(:,11);

flxa=x(:,12);
flxb=x(:,13);
flxabs=x(:,14);

flxa1=x(:,15);
flxb1=x(:,16);

flxa2=x(:,17);
flxb2=x(:,18);

flx_vcompa=x(:,19);
flx_vcompb=x(:,20);

flx_bemfa=x(:,21);
flx_bemfb=x(:,22);


pll_ang=x(:,23);
pll_we=x(:,24);
pll_pherror=x(:,25);

flx_ursa=x(:,26);
flx_ursb=x(:,27);



figure(1)
plot(i, ua, i, ub);
grid on
legend('ua','ub');

figure(2)
plot(i, ud, i, uq);
grid on
legend('ud','uq');

figure(3)
plot(i, ud_duty, i, uq_duty);
grid on
legend('ud duty','uq duty');

figure(4)
plot(i, ia, i, ib);
grid on
legend('ia','ib');

figure(5)
plot(i, id, i, iq);
grid on
legend('id','iq');

figure(6)
plot(i, flxa, i, flxb, i, flxabs, i, flxa1, i, flxb1, i, flxa2, i, flxb2 );
grid on
legend('flxa','flxb','flxabs','flxa1','flxb1','flxa2','flxb2');

figure(7)
plot(i, flx_vcompa, i, flx_vcompb);
grid on
legend('flx vca','flx vcb');

figure(7)
plot(i, flx_bemfa, i, flx_bemfb);
grid on
legend('bemfa','bemfb');


figure(8)
subplot(211)
plot(i, pll_ang);
grid on
legend('pll ang');

subplot(212)
plot(i, pll_we);
grid on
legend('pll we');

figure(9)
plot(i, pll_pherror);
grid on
legend('pll ph err');

figure(10)
plot(i, flx_ursa, i, flx_ursb);
grid on
legend('ursa','ursb');

