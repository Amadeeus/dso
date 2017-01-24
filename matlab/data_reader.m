clear all
close all
%% Read dataset
% Dataset directory
dataset_dir = '../build/bin/result/Robotcar_15_08_12_15_04_18_centre_01_2000/';

landmark_file = fopen([dataset_dir 'pointCloudPositions.txt'], 'r');
landmark_raw_data = fscanf(landmark_file, '%f', [9 Inf])';

keyframe_file = fopen([dataset_dir 'keyframePoses.txt'], 'r');
keyframe_raw_data = fscanf(keyframe_file, '%f', [9 Inf])';

%%
% Sort keyframes and landmarks in ascending order
[~, kf_arrange] = sort(keyframe_raw_data(:, 1));
[~, lm_arrange] = sort(landmark_raw_data(:, 1));

keyframe_sorted = keyframe_raw_data(kf_arrange, :);
landmark_sorted = landmark_raw_data(lm_arrange, :);

kf_positions = keyframe_sorted(:,3:5);
kf_orientations = keyframe_sorted(:,6:9);

lm_positions = landmark_sorted(:, 2:4);

% Outlier rejection
landmark_position_mean = mean(lm_positions);
landmark_position_std = std(lm_positions);

threshold = landmark_position_std;

deviation = abs(bsxfun(@minus, lm_positions, landmark_position_mean));
filter = sum(bsxfun(@ge, deviation, threshold), 2) == 0; 

lm_positions_filtered = lm_positions(filter, :);

% Align landmarks with gravity direction
[lm_positions_aligned, kf_positions_aligned, ~] = ...
    position_alignment(lm_positions_filtered, kf_positions, kf_orientations);

%% Plot
figure(1);
landmark = pointCloud(lm_positions_filtered);
pcshow(landmark);
hold on;
plot3(keyframe_sorted(:,3), keyframe_sorted(:,4), keyframe_sorted(:,5))

title('DSO Edge Map');
xlabel('X');
ylabel('Y');
zlabel('Z');

figure(2);
landmark_aligned = pointCloud(lm_positions_aligned);
pcshow(landmark_aligned);
hold on;
plot3(kf_positions_aligned(:,1), kf_positions_aligned(:,2), ...
    kf_positions_aligned(:,3))

title('DSO Edge Map (Aligned with gravity direction)');
xlabel('X');
ylabel('Y');
zlabel('Z');