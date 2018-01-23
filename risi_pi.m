%risi_pi.m

load pi.dat

time=pi(:,1);
out=pi(:,2);
integ=pi(:,3);


figure(1)
plot(time,out,time,integ/(1*32768));
grid on
legend('out','integrator');



