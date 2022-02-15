% testing the accelrometer
clear all
close("all")
ports = serialportlist;
pb = PyBench(ports(end));
N = 500;                    % 500 time points
end_time = 10.0;            % initial guess of time axis range
while true
    % plot axes
    figure(1)
    clf(1)
    axis([0 end_time -90 90]);
    title("Accelerometer: ptch & Roll Angles", "FontSize", 16);
    ylabel("Angles (deg)", "FontSize", 14);
    xlabel("Time (sex)", "FontSize", 14);
    grid on; hold on;
    tic;

    % read and plot acclerometer data
    for i = 1:N
        [p,r] = pb.get_accel(); % rad
        timestamp = toc;
        pitch = p * 180 / pi;   % convert to deg
        roll = r * 180 / pi;
        plot(timestamp, pitch, ".b");   % plot pitch in blue
        plot(timestamp, roll, ".r");    % roll in red
        pause(0.001);
    end
    end_time = toc;
end