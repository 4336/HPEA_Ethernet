clear; clc;
% y(n) = beta * y(n-1) + (1-beta) * x(n)
% y(n) = (1-alpha) * y(n-1) + alpha * x(n)
%% from Cutoff Freq [Hz]
lpf.cutoff = 10;
lpf.hz = 15000;
lpf.dt = 1/lpf.hz;
lpf.omega = 2*pi*lpf.cutoff; % cutoff freqency
lpf.beta = exp(-lpf.omega * lpf.dt);
lpf.alpha = 1-lpf.beta;

lpf

%% from alpha
lpf.alpha = 0.14;
lpf.hz = 500;

lpf.beta = 1-lpf.alpha;
lpf.dt = 1/lpf.hz;
lpf.omega = -log(lpf.beta) / lpf.dt;
lpf.cutoff = lpf.omega/(2*pi);

lpf

%% latency

fc = 12;
f = 12;
latency = 1/((1+(f/fc)^2)*fc)