[mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, ang_x, ang_y, ang_z, yaw, t] = loader("boston_mini_tour.bag");
[easting, northing, t_gps] = gps_loader("boston_mini_tour.bag");

%% Correcting for Hard and Soft Iron distortions

load Calib_data.mat;
[~,len] = size(mag_x);
hard_corrected_vec = [mag_x-ellipse.X0_in;mag_y-ellipse.Y0_in;zeros(1,len)];

new_vec = rotz(-ellipse.phi)*hard_corrected_vec;

corr_mag_x = new_vec(1,:);
corr_mag_y = new_vec(2,:);

%% Extracting heading from IMU

% The X and Y axis are switched according to the IMU axis
yaw_mag = atan2(corr_mag_x, corr_mag_y);
yaw_mag = wrapToPi(yaw_mag - yaw_mag(1));
%yaw_mag = wrapToPi(yaw_mag);

% Extracting heading from Gyro
gyro_heading = wrapToPi(cumtrapz(ang_z)*(1/40)); %+ yaw_mag(1);
% gyro_heading = cumtrapz(ang_z);

% Adding bias to yaw
%yaw = yaw;% + (yaw_mag(1) - yaw(1));

%% Complementary Filter

Fuse = complementaryFilter("SampleRate",40);

gy = [ang_x', ang_y', ang_z'];
mag_read = [corr_mag_x', corr_mag_y', mag_z'];
acc = [accel_x', accel_y', accel_z'];

q = Fuse(acc, gy, mag_read);
eul_val = quat2eul(q);
filt_yaw = -eul_val(:,1);

%% Integrating acceleration to get velocity

accel1_x = accel_x- accel_x(1);
vel_x = cumtrapz(accel1_x)*(1/40);

%% Extracting velocity from GPS

[~,len2] = size(easting);
vel_gps = zeros(1,len2);

for j = 2:len2
    diff_x = easting(j) - easting(j-1);
    diff_y = northing(j) - northing(j-1); 
    vel_gps(j) = (diff_x.^2 + diff_y.^2).^0.5;
end

