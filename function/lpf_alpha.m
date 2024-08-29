clc;
freq = linspace(1,500,100);
dt = 1/15000;
w = 2*pi*freq;

figure(2); clf;
subplot(1,2,1)
plot(freq, 1-exp(-w*dt))
hold on;
plot(freq, w*dt)
plot(freq, w*dt./(1+w*dt))
title('on 15kHz sampling frequency')
xlabel('cutoff frequency [Hz]')
ylabel('alpha')
l=legend('1-exp(-w*dt)', 'w*dt', 'w*dt/(1+w*dt)');
l.Location = 'northwest'

freq = linspace(1,500,100);
dt = 1/1000;
w = 2*pi*freq;

subplot(1,2,2)
plot(freq, 1-exp(-w*dt))
hold on;
plot(freq, w*dt)
plot(freq, w*dt./(1+w*dt))
title('on 1kHz sampling frequency')
xlabel('cutoff frequency [Hz]')
ylabel('alpha')
l=legend('1-exp(-w*dt)', 'w*dt', 'w*dt/(1+w*dt)');
l.Location = 'northwest'
