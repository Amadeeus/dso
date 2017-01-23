function pc_aligned = align_pointcloud(pc)
% Align the point cloud with gravity direction using PCA.

% Compute the mean of the point cloud
pc_mean = mean(pc, 1);

% Center the point cloud
pc_centered = bsxfun(@minus, pc, pc_mean);

%Computation of the principal directions of the point cloud
[V, ~] = eig(pc_centered' * pc_centered);

T_desired = [0 0 1; 0 1 0 ; -1 0 0];
pc_aligned = (T_desired * inv(V) * pc_centered')';
end