function [sig] = sine_gen(amp, f, fs, T)
% Function to generate a sinewave of amplitude amp, frequency f
% .... with a sampling frequency fs for a duration T

    dt = 1/fs;
    t = 0:dt:T;
    sig = amp*sin(2*pi*f*t);