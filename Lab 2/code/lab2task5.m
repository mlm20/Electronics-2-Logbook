% Lab 2 - Task 5 - Analyse two drum beats
%
clear all
[sig fs] = audioread("bass.wav");
sound(sig, fs)

% plot the signal

figure(1);
clf;
plot(sig);
xlabel("Sample no");
ylabel("Signal (v)");
title("Bass");

% Divide the signal into 20 msec segments,
% and computer the energy of the signal in
% that segment.

T = 0.02;       % 20 ms segments
N = fs * T;     % N samples
E = [];

for i = 1 : N : length(sig) - N + 1
    seg = sig(i : i + N - 1);
    E = [E seg' * seg];
end

% plot the energy graph and the peak values

figure(2);
clf;
x = 1:length(E);
plot(x, E)
xlabel("Segment number");
ylabel("Energy");
hold on

% find local maxima

[pks locs] = findpeaks(E);
plot(locs, pks, "o");
hold off

% plot spectrum of energy

figure(3)
plot_spec(E - mean(E), 1/T);





