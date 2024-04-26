max_grid_z =15;
min_grid_z =1;
x_grid_vector =[-1 0 0.4 0.8 1.2 1.6 2.0]; %in m
y_grid_vector =[-0.5 -0.1 0.1 0.5]; %in m
delta_z = max_grid_z-min_grid_z;

z_heights = [0 0 0 0;0 0 0 0;rand(4,4)*(delta_z) + min_grid_z;   0 0 0 0];
