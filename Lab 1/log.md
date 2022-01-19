# Lab 1 Log

## Matlab Installation

Matlab was installed per the instructions

## Exercise 1 - Sinusoidal signal generation

The code for `sine_gen` was copied from the lab notes

```matlab
function [sig] = sine_gen(amp, f, fs, T)
% Function to generate a sinewave of amplitude amp, frequency f
% .... with a sampling frequency fs for a duration T

    dt = 1/fs;
    t = 0:dt:T;
    sig = amp*sin(2*pi*f*t);
```

I applied this using the suggested parameters and generated the following plot

![](C:\Users\MaxLM\OneDrive\Documents\University\DE2\Electronics 2\Electronics-2-Logbook\Lab 1\media\Exercise 1 Graph.png)

I learnt that you can generate signals (and plot them) in matlab, that you can adjust parameters such as amplitude, sampling frequency, and duration.

