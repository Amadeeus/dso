clear all
close all
%% Dataset directory
dataset_dir = '../build/bin/';

pointcloud_file = fopen([dataset_dir 'pointCloudPositions.txt'], 'r');
raw_data = fscanf(pointcloud_file, '%f', [9 Inf])';
pointcloud = raw_data(:, 2:4);

threshold = 3;

for i = 1:2
    pc_mean = mean(pointcloud);
    pc_std = std(pointcloud);
    
    num_point = size(pointcloud, 1);
    filter = sum(abs(pointcloud - repmat(pc_mean, [num_point 1])) > repmat(threshold * pc_std, [num_point 1]) , 2) == 0;
    pointcloud = pointcloud(filter, :);
end

pc = pointCloud(pointcloud);
pcshow(pc);

title('DSO Edge Map');
xlabel('X');
ylabel('Y');
zlabel('Z');

pointcloud_aligned = align_pointcloud(pointcloud);
pc_aligned = pointCloud(pointcloud_aligned);
pcshow(pc_aligned);

title('DSO Edge Map (Aligned with gravity direction)');
xlabel('X');
ylabel('Y');
zlabel('Z');
