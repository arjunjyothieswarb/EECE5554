data_extraction
figure();
hold on;
plot(t, yaw_mag);
plot(t, gyro_heading);
plot(t, yaw);
plot(t,filt_yaw);

legend("Mag","Gyro","Yaw","Complementary Filter");
title("Yaw")
xlabel("Time");
ylabel("Yaw in rad");

figure();
hold on;
plot(t,vel_x);
plot(t_gps, vel_gps)
legend("IMU","GPS")

title("Velocity");
xlabel("Time in secs")
ylabel("Velocity in m/s")

figure();
plot(t,cumtrapz(accel_x)*(1/40));
title("Velocity from IMU(Uncorrected)")
xlabel("Time in secs");
ylabel("Velocity in m/s");

%% LPF and HPF

Fs = 40;
Fc = 15;
Wn = Fc/(Fs/2);

[b,a] = butter(4,Wn,'low');
mag_filt = filtfilt(b,a,yaw_mag);

Fc = 2;
Wn = Fc/(Fs/2);

[b,a] = butter(4,Wn,"high");
gyro_filt = filtfilt(b,a,gyro_heading);

figure();
hold on;
plot(t,gyro_filt);
plot(t,mag_filt);
plot(t,filt_yaw);
title("Filters")
legend("HPF-Gyro","LPF-Mag","CP Filter")
xlabel("Time in secs")
ylabel("Yaw in rad")