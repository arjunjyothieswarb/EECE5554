function [easting, northing,t_gps] = gps_loader(file_path)
    bag = rosbag(file_path);
    bsel = select(bag, 'Topic', '/gps');
    msgStructs = readMessages(bsel, 'DataFormat', 'struct');
    len = numel(msgStructs);

    t_gps = zeros(1,len);
    easting = zeros(1,len);
    northing = zeros(1,len); 
    
    for i = 1:len
        t_gps(i) = msgStructs{i}.Header.Stamp.Sec;
        easting(i) = msgStructs{i}.UTMEasting;
        northing(i) = msgStructs{i}.UTMNorthing;
    end

    t_gps = t_gps - t_gps(1);
end