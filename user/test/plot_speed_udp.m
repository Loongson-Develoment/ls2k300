clear;
clc;

localPort = 5005;
u = udpport("byte", "IPV4", "LocalPort", localPort);
configureTerminator(u, "LF");
flush(u);

t0 = tic;

figure("Name", "Position UDP Monitor");
targetLine = animatedline("Color", [0.85 0.1 0.1], "LineWidth", 1.5);
currentLine = animatedline("Color", [0.1 0.3 0.85], "LineWidth", 1.5);
grid on;
xlabel("Time (s)");
ylabel("Position (ml)");
legend("Target position", "Current position", "Location", "best");
title(sprintf("UDP position monitor, local port %d", localPort));

while ishandle(gcf)
    if u.NumBytesAvailable <= 0
        pause(0.005);
        continue;
    end

    line = strtrim(readline(u));
    values = sscanf(line, "%f,%f");
    if numel(values) ~= 2
        continue;
    end

    now = toc(t0);
    addpoints(targetLine, now, values(1));
    addpoints(currentLine, now, values(2));
    drawnow limitrate;
end

clear u;
