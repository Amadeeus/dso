clear all
close all
%% Read dataset
% Dataset directory
dataset_dir = '../build/bin/result/Robotcar_15_05_19_14_06_38_segment_01_2000_dataAssociation/';

% landmark_file = fopen([dataset_dir 'pointCloudPositions.txt'], 'r');
% landmark_raw_data = fscanf(landmark_file, '%f', [9 Inf])';

keyframe_file = fopen([dataset_dir 'keyframePoses.txt'], 'r');
keyframe_raw_data = fscanf(keyframe_file, '%f', [9 Inf])';

landmark_file = [dataset_dir 'test_associationTrack.txt'];
landmark_raw_data = import_data_association_file(landmark_file);

assert(sum(sum(~isnan(landmark_raw_data(:, end)))) == 0, ...
    'The predefined column number of landmark_raw_data matrix is too small!');
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

threshold = 2 * landmark_position_std;

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