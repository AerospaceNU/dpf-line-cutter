clc
clear

data = csvread("D:\Downloads\drone_test_3_darthplagueis.csv");
%data = data(6500:end,:);

alt = data(:,5);
time = data(:,2) / 1000.0;
state = data(:,1);
photores = data(:, 16);
accel = sqrt(data(:,9).^2+data(:,10).^2+data(:,11).^2);

tiledlayout(5, 1)

nexttile
plot(time,alt)
title("Smoothed Altitude over time")

nexttile
plot(time, photores)
title("Photores over time")

nexttile
plot(time, state)
title("State over time")

nexttile
plot(time, accel)
title("Accel over time")

nexttile
%plot(time)
%title("Time over time")
plot(time, data(:, 3))
title("Pressure over time")