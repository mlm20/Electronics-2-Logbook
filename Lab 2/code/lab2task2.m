% Lab 2 - Task 2 - Signal generation and capture with PyBench
%
clear all
ports = serialportlist;     % find all serial ports
pb = PyBench(ports(end));   % create a PyBench object with last port

% Set parameters

f = 440;                    % signal frequency
fs = 8000;                  % sampling frequency
pb = pb.set_sig_freq(f);
pb = pb.set_samp_freq(fs);
pb = pb.set_max_v(3.0);     % set maximum output voltage
pb = pb.set_min_v(0.5);     % set minimum output voltage
pb = pb.set_duty_cycle(50);

% Generate a signal

pb.triangle();

% Capture N samples

N = 1000;
samples = pb.get_block(N);
data = samples - mean(samples);

% plot data

figure(1);
plot(data(1:200),"o");
hold on
plot(data(1:200));
xlabel("Sample no");
ylabel("Signal Voltage (V)");
title("Captured signal");
hold off

% find spectrum

figure(2);
plot_spec(data, fs);