% measure system gain at f_sig
clear all
ports = serialportlist;
pb = PyBench(ports(end));

% generate a sine wave at sig_freq Hz
max_x = 1.55;
min_x = 1.45;
f_sig = 9.0;
pb = pb.set_sig_freq(f_sig);
pb = pb.set_max_v(max_x);
pb = pb.set_min_v(min_x);
pb.sine();
pause(2)

% capture output y(t)
pb = pb.set_samp_freq(100); % sample at 100 Hz
N = 300; % no samples
y = pb.get_block(N);

% plot signal
plot(y);
xlabel("Sample no.");
ylabel("Output voltage");
title("Bulb Box output (V)");

% Compute gain
x_pk2pk = max_x - min_x;
y_pk2pk = max(y) - min(y);
G = y_pk2pk / x_pk2pk;
G_dB = 20 * log10(y_pk2pk / x_pk2pk);