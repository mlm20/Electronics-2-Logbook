% testing the gyroscope

clear all
close("all")
ports = serialportlist;
pb = PyBench(ports(end));
N = 500; % 500 time points
end_time = 10.0; % initial guess of time axis range
gx = 0; gy = 0; % initialise pitch and roll

while true
    % plot axes first for plot later
    figure(1)
    clf(1)
    axis([0 end_time -90 90]);
    title("Gyroscope Pitch & Roll Angles", "FontSize", 16);
    ylabel("Angles (deg)", "FontSize", 14);
    xlabel("Time (sec)", "FontSize",14);
    grid on; hold on;
    timestamp = 0;
    tic;

    % read gyroscope data
    for i = 1:N
        [x, y, z] = pb.get_gyro(); % angular rate in rad/s
        dt = toc; % get elapsed time
        tic;
        timestamp = timestamp + dt;
        gx = max(min(gx + x * dt, pi/2), -pi/2); % limit to +- pi/2
        gy = max(min(gy + y * dt, pi/2), -pi/2);
        plot(timestamp, gy * 180 / pi, ".b"); % plot pith in blue
        plot(timestamp, gx * 180 / pi, ".r"); % plot pith in red
        pause(0.001);
    end
    end_time = timestamp;
end