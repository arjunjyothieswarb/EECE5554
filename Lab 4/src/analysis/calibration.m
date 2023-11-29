[mag_x, mag_y, ang_z] = loader("circle_data.bag");
ellipse = fit_ellipse(mag_x, mag_y);
[~,len] = size(mag_x);
hard_corrected_vec = [mag_x-ellipse.X0_in;mag_y-ellipse.Y0_in;zeros(1,len)];
new_vec = rotz(-ellipse.phi)*hard_corrected_vec;
% plotter