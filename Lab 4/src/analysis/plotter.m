figure()
plot(new_vec(1,:), new_vec(2,:))
vertical_lim = ylim();
vertical_y = linspace(vertical_lim(1,1), vertical_lim(1,2), 50);
vertical_x = zeros(1,50);
hold on;
plot(vertical_x, vertical_y)

horizontal_lim = xlim();
horizontal_y = zeros(1,50);
horizontal_x = linspace(2*horizontal_lim(1,1), -horizontal_lim(1,1),50);
plot(horizontal_x, horizontal_y);