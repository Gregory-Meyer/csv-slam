clear();
close('all');
clc();

groundtruth_pathname = '~/Downloads/nclt/groundtruth_2012-11-16.csv';
odometry_pathname = '~/Downloads/nclt/2012-11-16/odometry_mu_100hz.csv';

input_parents = {'.', '.'};
input_filenames = {
    '3d+++rotation+++translation.csv', ...
    '2d+++rotation+++translation.csv'
};

input_pathnames = cellfun(...
    @(parent, filename) fullfile(parent, filename), ...
    input_parents, ...
    input_filenames, ...
    'UniformOutput', false ...
);

input_tables = cellfun(...
    @(filename) readtable(filename), ...
    input_pathnames, ...
    'UniformOutput', false ...
);

for i = 1:numel(input_tables)
    this_input_table = input_tables{i};
    this_input_table.x = -this_input_table.x;
    input_tables{i} = trim_rotate(this_input_table);
end

variable_names = input_tables{1}.Properties.VariableNames;

hold('on');

groundtruth_table = readtable(groundtruth_pathname);
groundtruth_table.Properties.VariableNames = variable_names;
groundtruth_table = trim_rotate(groundtruth_table);

plot3(groundtruth_table.x, groundtruth_table.y, groundtruth_table.z);

odometry_table = readtable(odometry_pathname);
odometry_table.Properties.VariableNames = variable_names;
odometry_table = trim_rotate(odometry_table);

plot3(odometry_table.x, odometry_table.y, odometry_table.z);

for i = 1:numel(input_tables)
    this_input_table = input_tables{i};

    plot3(this_input_table.x, this_input_table.y, this_input_table.z);
end

labels = {'ground truth', 'odometry', 'cartographer 3d', 'cartographer 2d'};

hold('off');
legend(labels);
axis('equal');

grid('on');
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
ax = gca();
ax.FontSize = 20;

print('-dpng', '../error-top.png');

view(0, 0);
print('-dpng', '../error-side.png');

tables = [{groundtruth_table}, {odometry_table}, input_tables];
xyz = cell(size(tables));

for i = 1:numel(tables)
    t = tables{i};

    utime_offset_s = (t.utime - t.utime(1)) * 1e-6;
    this_xyz = zeros(121, 3);

    for j = 0:120
        [~, closest_row] = min(abs(utime_offset_s - j));
        row = t(closest_row, :);

        this_xyz(j + 1, :) = [row.x, row.y, row.z];
    end

    xyz{i} = this_xyz;
end

figure();
hold('on');

for i = 1:numel(tables)
    xyz_offset = xyz{i} - xyz{1};
    xyz_error = sqrt(sum(xyz_offset.^2, 2));

    fprintf('%s:\n', labels{i});
    fprintf('  - mean xyz error: %f\n', mean(xyz_error));
    fprintf('  - final xyz error: %f\n', xyz_error(end));

    plot(0:120, xyz_error);
end

hold('off');
xlabel('time (s)')
ylabel('3d error (m)');
grid('on');
legend(labels);
ax = gca();
ax.FontSize = 20;
print('-dpng', '../3d-translation-error.png');

figure();
hold('on');

for i = 1:numel(tables)
    xyz_offset = xyz{i} - xyz{1};
    xy_offset = xyz_offset(:, 1:2);
    xy_error = sqrt(sum(xy_offset.^2, 2));

    fprintf('%s:\n', labels{i});
    fprintf('  - mean xy error: %f\n', mean(xy_error));
    fprintf('  - final xy error: %f\n', xy_error(end));

    plot(0:120, xy_error);
end

hold('off');
xlabel('time (s)')
ylabel('2d error (m)');
grid('on');
legend(labels);
ax = gca();
ax.FontSize = 20;
print('-dpng', '../2d-translation-error.png');

function u = trim_rotate(t)
    start_time = 660;
    duration = 120;
    utime_offset_s = (t.utime - t.utime(1)) * 1e-6;

    u = t(utime_offset_s >= start_time & utime_offset_s <= start_time + duration, :);

    xy = [u.x - u.x(1), u.y - u.y(1)];

    initial_angle = atan2(xy(2, 2) - xy(1, 2), xy(2, 1) - xy(1, 1));
    xy = xy * R(initial_angle);

    u.x = xy(:, 1);
    u.y = xy(:, 2);
    u.z = u.z - u.z(1);
end

function A = R(theta)
    A = [cos(theta), -sin(theta); sin(theta), cos(theta)];
end
