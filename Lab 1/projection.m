s1 = sine_gen(1.0, 400, 10000, 1);
s2 = sine_gen(0.5, 1000, 10000, 1);
s3 = sine_gen(1.0, 401, 10000, 1);

dot_product = s1 * s2';
dot_product_2 = s1 * s3';
dot_product_3 = (s1 + s2) * s1';

disp(dot_product)
disp(dot_product_2)
disp(dot_product_3)