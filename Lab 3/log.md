# Lab 3 Log

[TOC]

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



### DC Characteristic using Matlab

The following code was used to generate the DC characteristic using Matlab.

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

