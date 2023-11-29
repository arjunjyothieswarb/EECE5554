data_extraction;
imu_y_accel = ang_z .* vel_x;

figure();
hold on;

plot(t, imu_y_accel);
plot(t, accel_y);

title("Acceleration in Y axis");
legend("Xw","Y''");
xlabel("Time in secs");
ylabel("Velocity in m/s")

vel_vec = zeros(3,len);

for k = 1:len
    vel_vec(:,k) = [vel_x(k);0;0];
    vel_vec(:,k) = rotz(yaw_mag(k))*vel_vec(:,k);
end

step_north(1) = northing(1);
step_east(1) = easting(1);

for i = 2:len
    step_north(i) = step_north(i-1) + vel_vec(1,i-1)*(1/40);
    step_east(i) = step_east(i-1) + vel_vec(2,i-1)*(1/40);
end

figure();
hold on;
plot(step_east, step_north)
plot(easting, northing)
title("Dead Reckoning")
legend("IMU","GPS")
xlabel("Easting in m")
ylabel("Northing")