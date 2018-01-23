%risi.m

load pll.dat

i=pll(:,1);
ang0=pll(:,2);
pll_ang=pll(:,3);
we=pll(:,4);
ph_err=pll(:,5);
inp_re=pll(:,6);
inp_im=pll(:,7);



figure(1)
plot(i, we);
grid on
legend('i');

figure(2)
plot(i,ang0,i,pll_ang,i,ph_err);
grid on
legend('ang0','pll_ang','ph_err');

figure(3)
plot(i,inp_re,i,inp_im)
grid on