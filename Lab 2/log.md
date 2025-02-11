

# Lab 2 Log

## Task 1 - Connecting the PyBench board to your PC or Macbook running Matlab

The PyBench was connected to my computer and was recognised as a device. The list of constituent python files was seen.

![](media/python.jpg)

The *PyBench Self-Test* was activated and all tests (apart from the motor test) were performed. They all passed and functioned as expected.

PyBench was setup with Matlab and I was able to access the *PyBench* class library properly.

```matlab
>> ports = serialportlist

ports = 

  1×2 string array

    "COM3"    "COM5"

>> pb = PyBench(ports(end))

pb = 

  PyBench with properties:

    BUFFERSIZE: 20000
      sig_freq: 10
          dc_v: 1.6500
         max_v: 3.3000
         min_v: 0
    duty_cycle: 50
     samp_freq: 100

>> 
```

## Task 2 - Using PyBench to generate signals via Matlab and explore their spectra

### Scope

The code given was used to generate a sine signal on the PyBench. This initially did not work, giving a strange signal on the scope but I discovered that the main ground pin on the PyBench was broken. When an alternative ground pin was used, the scope displayed the expected waveform.

```matlab
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

pb.sine()
```

![](media/scope1.jpeg)

By changing `pb.sine()` to `pb.square()` and `pb.triangle()` I was able to generate square and triangle waveforms.

![](media/scope2.jpeg)

![](media/scope3.jpeg)

### Sampling

The code was amended was added the following lines to the end of the existing code.

```matlab
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
```

This samples the signals generated by the *PyBench* were sampled by *Matlab* and plots the frequency spectrum for each.

![](media/spectrum1.png)

*Sine spectrum*

![](media/spectrum2.png)

*Square wave spectrum*

![](media/spectrum3.png)

*Triangle wave spectrum*

The sine wave has a single consistent frequency and thus only has a single spike in its spectrum.

The square wave is a superposition of many sine waves and thus has many spikes.

The triangle wave is a superposition of of sine waves too but has a more obvious frequency compared to the square waves, which is why its first spike is more prominent in comparison to subsequent spikes. Compared to the square wave spectrum.

## Task 3 - Capture and analyse microphone signals

The following Matlab script was used to produce a time-domain and frequency-domain plot of the signal captured from the PyBoard’s microphone. 

```matlab
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
```

It was tested on our whistling as well as a 2000 Hz sound from a tuning fork app. The plots from the tuning fork app are shown below.

![](media/task3fig1.png)

![](media/task3fig2.png)

### Spectrum Analyser

The following code was added to make this process into a continuously updating frequency analyser.

```matlab
% repeat capture and plot spectrum
while true
    samples = pb.get_mic(N);
    data = samples - mean(samples);
    figure(2)
    clf;
    plot_spec(data,fs);
end
```

The following frequency spectrum was generated from my lab partner singing, harmonics are visible.

![](media/task3fig3.png)

## Task 4 - Windowing effect on a signal

The code from the previous task was modified to use the `plot_spec_dB()` function instead of `plot_spec` to plot a window of the function using the logarithmic Decibel scale.

The following plots were obtained for *1000 Hz* and *1100 Hz* respectively.

![](media/task4fig1.png)

*1000 Hz*

![](media/task4fig2.png)

*1100 Hz*

The two signals generated, despite only having a frequency difference of 100 Hz are very different. This difference is due to the *windowing effect*.

### Hamming window

The spectrum generator was amended to be the following code in order to generate a Hamming window.

```matlab
% find and plot spectrum
figure(2)
plot_spec_(data, fs)

% create a hamming window
window = hamming(length(data));
while true
    samples = pb.get_mic(N);
    data = samples - mean(samples);
    clf;
    plot_spec_dB(data,fs);
    hold on
    plot_spec_dB(data.*window, fs);
end
```

This generated the following plots for the same frequencies

![](media/task4fig3.png)

*1000 Hz*

![](media/task4fig4.png)

*1100 Hz*

The signals with and without the Hamming Window are almost identical for *1000 Hz* however there is a significant difference for *1100 Hz*

## Task 5 - Music signal segmentation and analysis

In order to perofrm the music analysis the following code was used

```matlab
% Lab 2 - Task 5 - Analyse two drum beats
%
clear all
[sig fs] = audioread("two_drums.wav");
sound(sig, fs)

% plot the signal

figure(1);
clf;
plot(sig);
xlabel("Sample no");
ylabel("Signal (v)");
title("Two Drums");

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
```

This generated the following graphs.

![](media/task5fig1.png)

*Time domain voltage plot of signal*

![](media/task5fig2.png)

*Time domain energy plot*

![](media/task5fig3.png)

*Frequency domain energy plot*

## Task 6 – Analysing complex music

