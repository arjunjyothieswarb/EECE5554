bag = rosbag("LocationD.bag");
bsel = select(bag, 'Topic','/vectornav');
msgStructs = readMessages(bsel, 'DataFormat','struct');

for i=1:numel(msgStructs)
    data = split(msgStructs{i}.Data, ',');
    if length(data) == 13
        angular_velx(i) = str2double(data(11));
        angular_vely(i) = str2double(data(12));
        gyro_z = split(data(13),'*');
        angular_velz(i) = str2double(gyro_z(1));

        lin_accx(i) = str2double(data(8));
        lin_accy(i) = str2double(data(9));
        lin_accz(i) = str2double(data(10));
    end
end