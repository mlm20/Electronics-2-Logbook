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

![](<media/Exercise 1 Graph.png>)

I learnt that you can generate signals (and plot them) in matlab, that you can adjust parameters such as amplitude, sampling frequency, and duration.

## Exercise 2 - Spectrum of the signal

The following code was used from the notes

```matlab
function plot_spec(sig, fs)

% Function to plot frequency spectrum of sig
%   usage:
%           plot_spectrum(sig, 8192)

    magnitude = abs(fft(sig));
    N = length(sig);
    df = fs/N;
    f = 0:df:fs/2;
    Y = magnitude(1:length(f));
    plot(f, 2*Y/N)
    xlabel("\fontsize{14}frequency (Hz)")
    ylabel("\fontsize{14}Magnitude")
```

I used this function to generate a plot of the Fourier Transform of the signal in exercise 1

![](<media/Exercise 2.png>)

This shows a spike at `f = 400`, the frequency of the original signal. This is the expected behaviour of a Fourier Transform.

## Exercise 3 - Two tones

The two sinewaves, s1 at 400Hz (amplitude 1.0V) and s2 at 1000Hz (amplitude 0.5V) were added and plotted using the following code

```matlab
s1 = sine_gen(1.0, 400, 10000, 1);
s2 = sine_gen(0.5, 1000, 10000, 1);
sig = s1 + s2;

plot(sig(1:200));

xlabel("\fontsize{14}Sample Number")
ylabel("\fontsize{14}Amplitude")
```

This generated the following plot

![](<media/Exercise 3.png>)

## Exercise 4 - Two tones + noise

Noise was added to the signal above

```matlab
s1 = sine_gen(1.0, 400, 10000, 1);
s2 = sine_gen(0.5, 1000, 10000, 1);
sig = s1 + s2;
noisy = sig + randn(size(sig));

plot(noisy(1:200));

xlabel("\fontsize{14}Sample Number")
ylabel("\fontsize{14}Amplitude")
```

![](<media/Exercise 4.png>)

![](<media/Exercise 4 Transform.png>)

From this I learnt that noise can be added to clean signals via the `randn()` function. The number of noise samples is determined by the number of samples in the original signal. The result is that the data becomes more detailed - more samples are added to the data array, but the trend still mimics the previous plot.

In the Fourier Transform however you can clearly see the two spikes showing the two signal frequencies. The noise is at a far lower intensity and thus can easily be ignored.

## Exercise 5: Projection using dot product

