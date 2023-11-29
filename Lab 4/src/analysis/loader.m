function [mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, ang_x, ang_y, ang_z, yaw, t] = loader(file_path)
    bag = rosbag(file_path);
    bsel = select(bag, 'Topic', '/imu');
    msgStructs = readMessages(bsel, 'DataFormat', 'struct');

    len = numel(msgStructs);

    mag_x = zeros(1,len);
    mag_y = zeros(1,len);
    mag_z = zeros(1,len);

    ang_x = zeros(1,len);
    ang_y = zeros(1,len);
    ang_z = zeros(1,len);
    
    accel_x = zeros(1,len);
    accel_y = zeros(1,len);
    accel_z = zeros(1,len);

    yaw = zeros(1,len);
    t = zeros(1,len);

    for i = 1:len
        mag_x(i) = msgStructs{i}.MagField.MagneticField_.X;
        mag_y(i) = msgStructs{i}.MagField.MagneticField_.Y;
        mag_z(i) = msgStructs{i}.MagField.MagneticField_.Z;
        
        ang_x(i) = msgStructs{i}.IMU.AngularVelocity.X;
        ang_y(i) = msgStructs{i}.IMU.AngularVelocity.Y;
        ang_z(i) = msgStructs{i}.IMU.AngularVelocity.Z;

        accel_x(i) = msgStructs{i}.IMU.LinearAcceleration.X;
        accel_y(i) = msgStructs{i}.IMU.LinearAcceleration.Y;
        accel_z(i) = msgStructs{i}.IMU.LinearAcceleration.Z;

        t(i) = msgStructs{i}.Header.Stamp.Sec + (msgStructs{i}.Header.Stamp.Nsec*(10^-9));

        val = split(msgStructs{i}.Raw,',');
        yaw(i) = str2double(val(2)) * pi/180;
    end
    t = t - t(1);
end