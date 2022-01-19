s1 = sine_gen(1.0, 400, 10000, 1);
s2 = sine_gen(0.5, 1000, 10000, 1);
s3 = sine_gen(1.0, 401, 10000, 1);

dot_product = s1 * s2';





xlabel("\fontsize{14}Sample Number")
ylabel("\fontsize{14}Amplitude")