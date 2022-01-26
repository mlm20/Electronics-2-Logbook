% Lab 2 - Task 3 - Capture and analyse microphone sound signal
%
clear all
ports = serialportlist;
pb = PyBench(ports(end));

% Set sampling frequency
fs = 8000;
pb = pb.set_samp_freq(fs);

% Capture N samples
N = 1000;
samples = pb.get_mic(N);
data = samples - mean(samples);

% plot data
figure(1);
clf
plot(data);
xlabel("Sample no")
ylabel("Signal voltage");
title("Microphone signal");

% find and plot spectrum
figure(2)
plot_spec(data, fs)

% repeat capture and plot spectrum
while true
    samples = pb.get_mic(N);
    data = samples - mean(samples);
    figure(2)
    clf;
    plot_spec(data,fs);
end