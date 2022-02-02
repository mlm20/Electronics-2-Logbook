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



