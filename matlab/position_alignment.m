function [lm_positions_aligned, kf_positions_aligned, ...
    kf_orientations_aligned] = position_alignment(lm_positions, ...
    kf_positions, kf_orientations)

% Align the point cloud with gravity direction using PCA.

% Compute the mean of the point cloud
lm_positions_mean = mean(lm_positions, 1);

% Center the point cloud
lm_positions_centered = bsxfun(@minus, lm_positions, lm_positions_mean);

% Computation of the principal directions of the point cloud
[T_C_lm, ~] = eig(lm_positions_centered' * lm_positions_centered);

% Compute transform from current frame to world frame (aligned with gravity).
T_W_lm = [0 0 1; 0 1 0 ; -1 0 0];
T_W_C = T_W_lm * inv(T_C_lm);   

% Align keyframes position and landmarks position in gravity direction.
lm_positions_aligned = (T_W_C * lm_positions')';
kf_positions_aligned = (T_W_C * kf_positions')';

num_keyframes = size(kf_orientations, 1);

kf_orientations_rotm = quat2rotm(kf_orientations);
kf_orientations_aligned = zeros(num_keyframes, 4);

for i = 1 : num_keyframes
    kf_orientations_aligned(i, :) = rotm2quat(T_W_C * kf_orientations_rotm(:,:,i));
end

end