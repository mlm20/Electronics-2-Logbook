# Lab 3 Log

## Setup

The Bulb Board was setup to the PyBench as described. The potentiometer was turned until the PyBench displayed ***1.97 V*** - between *1.9* and *2.1 V*.

## Task 1 - DC Characteristic of the Bulb Board

The following code was inputted into the command window

```matlab
>> clear all
>> ports = serialportlist

ports = 

    "COM4"

>> pb = PyBench(ports(end));
>> pb.dc(1.5);
>> pause(1)
>> pb.get_one()

ans =

   2.0125
```

When the output voltage was set to *0V*, the bulb board returned *0.1152V* - this is very close to zero as expected.

By testing `x_dcH` was found to be between 1.75 and 2V.

`x_dcL` was found to be 0.4 V (very dim).

### `y_dc` vs `x_dc` graph

The following values were generated for the graph (between `x_dcL` and `x_dcH`)

| Applied Voltage | Light Intensity |
| --------------- | --------------- |
| 0.4             | 0.112           |
| 0.8             | 0.3207          |
| 1.2             | 1.0498          |
| 1.6             | 2.5862          |
| 2               | 3.2992          |
| 2.4             | 3.2992          |

![](media/Graph1.png)



### DC Characteristic using MATLAB

The following code was used to generate the DC characteristic using MATLAB.

```matlab
% lab 3 task 1
ports = serialportlist;
pb = PyBench(ports(end));

pb.samp_freq = 100;
NSTEPS=25;
input = zeros(NSTEPS);
output = zeros(NSTEPS,1);
tic
disp('SWEEPING DRIVE VOLTAGE FOR DC STEADY-STATE CHARACTERISTICS');
for i = [1:NSTEPS]
    v = (i-1)*2.5/NSTEPS;
    input(i) = v;
    pb.dc(v);
    pause(0.5);
    data = pb.get_block(10);
    output(i) = mean(data);
end
pb.dc(0.0);
toc % stop timer tic toc
figure
plot(input, output)
xlabel('Input (V)');
ylabel('Output (V)');
title('DC input output transfer function');
fclose(instrfind());
```

This generated the following plot.

![](<media/DC INPUT OUTPUT TRANSFOR FUNCT.png>)

The system is non-linear because `P = V^2 / R` which implies a parabolic relationship. If the system were linear we could see a straight line.

The system is approximated linear from 0 to 1.5 and from roughly 1.3 to 1.6.

## Task 2 - Frequency Response of the Bulb Board system – Theoretical only

The following supplied code was typed out, this generated the following graph.

```matlab
% lab3 task 2

f = (0:0.1:20);
D = [0.038 1.19 43 1000];
s = 1i * 2 * pi * f;
G = 1000 ./ abs(polyval(D,s));
Gdb = 20 * log10(G);

figure;
plot(f,Gdb);
xlabel("Freq Hz")
ylabel("Gain dB")
title("frequency response - theoretical");
```

![](media/frequency_response.png)

The frequency response shows that 5V is the resonant frequency, and more widely the behaviour of the bulb board at different frequencies.

The following values for G(s) were calculated

| Frequency (Hz) | G(s)          |
| -------------- | ------------- |
| 0              | 0             |
| 5              | 12.2 - 20.5j  |
| 20             | -37.2 + 15.8j |

These are complex numbers, but if we take the real components these line up with the frequency response generated above.
