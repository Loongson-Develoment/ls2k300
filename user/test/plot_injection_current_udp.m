clear;
clc;

localPort = 5006;
u = udpport("byte", "IPV4", "LocalPort", localPort);
configureTerminator(u, "LF");
flush(u);

t0 = tic;

figure("Name", "Injection Bus Current Contact Monitor");
tiledlayout(3, 1);

nexttile;
filteredCurrentLine = animatedline("Color", [0.1 0.55 0.2], "LineWidth", 1.2);
grid on;
xlabel("Time (s)");
ylabel("Filtered current (raw unit)");
title("Filtered bus current");

nexttile;
movingAverageLine = animatedline("Color", [0.1 0.3 0.85], "LineWidth", 1.2);
grid on;
xlabel("Time (s)");
ylabel("Current (raw unit)");
title("500 ms moving average");

nexttile;
baselineDeltaLine = animatedline("Color", [0.85 0.1 0.1], "LineWidth", 1.2);
grid on;
xlabel("Time (s)");
ylabel("Current delta (raw unit)");
title("Moving average minus baseline");

while ishandle(gcf)
    if u.NumBytesAvailable <= 0
        pause(0.005);
        continue;
    end

    packet = strtrim(readline(u));
    values = sscanf(packet, "%f,%f,%f");
    if numel(values) ~= 3
        continue;
    end

    now = toc(t0);
    addpoints(filteredCurrentLine, now, values(1));
    addpoints(movingAverageLine, now, values(2));
    addpoints(baselineDeltaLine, now, values(3));
    drawnow limitrate;
end

clear u;
