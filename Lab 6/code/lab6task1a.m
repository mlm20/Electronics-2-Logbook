clear all
[sig fs] = audioread("bgs.wav");

% add noise to music
x = sig + 0.2 * rand(size(sig));

% plot the signal
figure(1);
clf;
plot(x);
xlabel("Sample no");
ylabel("Signal (v)");
title("Stay Alive music");

% filter music with moving average filter
N = size(x);
for i = 4 : N
    y(i) = (x(i) + x(i-1) + x(i-2) + x(i-3))/4;
end

y(1) = x(1) / 4;
y(2) = (x(2) + x(1) / 4);
y(3) = (x(3) + x(2) + x(1))/4;

y(u) = 

% play original & then the filtered sound

sound(x, fs)
disp("playing the original - press enter when done")
pause;
sound(y, fs)
disp("playing the filter music")