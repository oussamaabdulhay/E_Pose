clc
clear all
close all
%%
folderPath = 'D:/data_collected/mustard/';
PC1 = importdata('D:/mustard/mustard.xyz');
outputDir = 'D:/RGB/mustard';
fileList = dir(fullfile(folderPath, '*.bag'));
fileNames = {fileList.name};
T_obj_r_base =  [0.88022 -0.391489 0.28198 0.04581;
            0.2504  -0.12227 -0.95919 -0.7345;
            0.40410  0.9196 -0.00596 0.0470;
            0 0 0 1];
cls_index = 6; 
for o = 1:length(fileNames)
% o = 1;
o
subDir = fullfile(outputDir, fileNames{o}(1:end-4));
ObjBoxFile = fullfile(subDir, 'mustard_box.txt');
if ~exist(subDir, 'dir')
    mkdir(subDir);
end
o
%% importing the Rosbag file
bag_file_path = [folderPath, fileNames{o}];
bag=rosbag(bag_file_path);
bag.AvailableTopics;
%%
dvs_image_topic=select(bag,'Topic', '/dvs/image_raw');  %, 'Time', time_range); 
dvs_image_msgs = readMessages(dvs_image_topic,'DataFormat','struct'); % topic & msg of DAVIS

dvs_pose_topic = select(bag, 'Topic', '/dvs/pose', ...);%,...
     'Time',[dvs_image_topic.StartTime dvs_image_topic.EndTime]); %, 'Time', time_range); 
dvs_pose = readMessages(dvs_pose_topic,'DataFormat','struct'); % topic & msg of DAVIS_pose

%%
image_width = dvs_image_msgs{1,1}.Width;
image_height = dvs_image_msgs{1,1}.Height;
img = cell(length(dvs_image_msgs), 2);
for i = 1:length(dvs_image_msgs)
    while length(dvs_image_msgs{i,1}.Data)<269880
        dvs_image_msgs{i,1}.Data= [dvs_image_msgs{i,1}.Data;0];
    end
    image_temp = dvs_image_msgs{i,1}.Data;
    img_tmp = reshape(image_temp, [3, image_width, image_height]);
    img{i,1} = permute(img_tmp, [3, 2, 1]);
    timeo = rostime(dvs_image_msgs{i}.Header.Stamp.Sec, dvs_image_msgs{i}.Header.Stamp.Nsec);
    img{i,2} = seconds(timeo);
end
%%
for i=1:length(dvs_pose)
    camera_pose(i,1) = dvs_pose{i}.Pose.Position.X;
    camera_pose(i,2) = dvs_pose{i}.Pose.Position.Y;
    camera_pose(i,3) = dvs_pose{i}.Pose.Position.Z;
    camera_pose(i,4) = dvs_pose{i}.Pose.Orientation.W;
    camera_pose(i,5) = dvs_pose{i}.Pose.Orientation.X;
    camera_pose(i,6) = dvs_pose{i}.Pose.Orientation.Y;
    camera_pose(i,7) = dvs_pose{i}.Pose.Orientation.Z;
    camera_position_time = rostime(dvs_pose{i}.Header.Stamp.Sec, dvs_pose{i}.Header.Stamp.Nsec);
    camera_pose(i,8) = seconds(camera_position_time);
end
%%
camera_pose(:,8) = camera_pose(:,8) - img{1,2};
for i = 2:length(img)
    img{i,2}=img{i,2} - img{1,2};
end
img{1,2} = 0;

%%
cut_off = camera_pose(1,8);
last_off = camera_pose(end,8);
timeColumn = img(:, 2);
numericArray = [timeColumn{:}];
cut_off_index = find(numericArray >= cut_off, 1, 'first');
last_index = find(numericArray >= last_off, 1, 'last');
img(1:cut_off_index,:)=[];
%%
for i=1:length(img)
    frame_times(i) = img{i,2};
end
%% interpolating pose data
for i=1:length(img)
    pose_before = find(camera_pose(:,8) <= img{i,2} , 1, 'Last');
    pose_after = find(camera_pose(:,8) > img{i,2} , 1, 'first');
    
    if pose_after ~= 1
        time_before = camera_pose(pose_before,8);
        time_after = camera_pose(pose_after,8);
        time_current = img{i,2};
        pn = quatnormalize([camera_pose(pose_before,4),camera_pose(pose_before,5),camera_pose(pose_before,6),camera_pose(pose_before,7)]);
        qn = quatnormalize([camera_pose(pose_after,4),camera_pose(pose_after,5),camera_pose(pose_after,6),camera_pose(pose_after,7)]);
        scale = (time_current - time_before)/(time_after - time_before);
        qi(i,:) = quatinterp(pn,qn,scale,'slerp');
    else
        qi(i,:) = pn;
    end
end

camera_position_upsampled(:,1) = interp1(camera_pose(:,8),camera_pose(:,1) , frame_times, 'linear');
camera_position_upsampled(:,2) = interp1(camera_pose(:,8),camera_pose(:,2) , frame_times, 'linear');
camera_position_upsampled(:,3) = interp1(camera_pose(:,8),camera_pose(:,3) , frame_times, 'linear');
%%
PC1 = PC1(:,1:3);
%%
imagesize = [260 346];
distCoeffs = [-0.362 0.115 0.0009576 0.00000455 0.130];
intrinsicMatrix = [292.52 0.0 172.61;
                        0. 293.19  130.125;
                        0.  0.  1.];

intrinsics = cameraIntrinsicsFromOpenCV(intrinsicMatrix,...
                                         distCoeffs,imagesize);
for i=1:length(img)
    undistorted_frames{i,1} = undistortImage(img{i,1},intrinsics);
end
%%
rotm = quat2rotm(qi);
for i=1:length(camera_position_upsampled)
    T_dvs_r_base(:,:,i) = [rotm(:,:,i) camera_position_upsampled(i,:)'];
end
for i=1:length(camera_position_upsampled)
    T_base_r_dvs (:,:,i) = inv([T_dvs_r_base(:,:,i);0 0 0 1]);
    T_object_r_dvs(:,:,i) =  T_base_r_dvs (:,:,i) * T_obj_r_base;
    T_dvs_r_object(:,:,i) = inv([T_object_r_dvs(:,:,i)]);
end
%%
for i=1:length(qi)
    allData{i,1} = img{i,1};
    allData{i,2} = undistorted_frames{i,1};
    allData{i,3} = T_object_r_dvs(:,:,i);
    allData{i,4} = img{i,2};
end
%%
for j=1:length(allData)
    R_test = allData{j,3}(1:3,1:3);
    t_test = allData{j,3}(1:3,4);
    focalLength = [intrinsicMatrix(1,1), intrinsicMatrix(2,2)]; 
    principalPoint = [intrinsicMatrix(1,3), intrinsicMatrix(2,3)]; 
    p_1 = [max(PC1(:,1)) + 0.001;max(PC1(:,2))+ 0.001;min(PC1(:,3))+ 0.001;1];
    p_2 = [max(PC1(:,1))+ 0.001;min(PC1(:,2))+ 0.001;min(PC1(:,3))+ 0.001;1];
    p_3 = [min(PC1(:,1))+ 0.001;max(PC1(:,2))+ 0.001;min(PC1(:,3))+ 0.001;1];
    p_4 = [min(PC1(:,1))+ 0.001;min(PC1(:,2))+ 0.001;min(PC1(:,3))+ 0.001;1];
    p_5 = [max(PC1(:,1))+ 0.001;max(PC1(:,2))+ 0.001;max(PC1(:,3))+ 0.001;1];
    p_6 =[max(PC1(:,1))+ 0.001;min(PC1(:,2))+ 0.001;max(PC1(:,3))+ 0.001;1];
    p_7 = [min(PC1(:,1))+ 0.001;max(PC1(:,2))+ 0.001;max(PC1(:,3))+ 0.001;1];
    p_8 =[min(PC1(:,1))+ 0.001;min(PC1(:,2))+ 0.001;max(PC1(:,3))+ 0.001;1];
    
    
    p_9 = [0;0;0;1];
    
    T_cam_r_obj = [R_test t_test;0 0 0 1];
    
    cam_point1 = (T_cam_r_obj) * p_1 ;
    cam_point2 =(T_cam_r_obj) * p_2 ;
    cam_point3 = (T_cam_r_obj) * p_3 ;
    cam_point4 = (T_cam_r_obj) * p_4 ;
    cam_point5 = (T_cam_r_obj) * p_5 ;
    cam_point6 =(T_cam_r_obj) * p_6 ;
    cam_point7 = (T_cam_r_obj) * p_7 ;
    cam_point8 = (T_cam_r_obj) * p_8 ;
    cam_point9 = (T_cam_r_obj) * p_9 ;
    
    pixel_point1 = [intrinsicMatrix(1,1) * cam_point1(1) / (cam_point1(3)) + intrinsicMatrix(1,3), intrinsicMatrix(2,2) * cam_point1(2) / (cam_point1(3)) + intrinsicMatrix(2,3)];%intrinsic_matrix * cam_point1';
    pixel_point2 = [intrinsicMatrix(1,1) * cam_point2(1) / (cam_point1(3)) + intrinsicMatrix(1,3), intrinsicMatrix(2,2) * cam_point2(2) / (cam_point1(3)) + intrinsicMatrix(2,3)];%intrinsic_matrix * cam_point2';
    pixel_point3 = [intrinsicMatrix(1,1) * cam_point3(1) / (cam_point1(3)) + intrinsicMatrix(1,3), intrinsicMatrix(2,2) * cam_point3(2) / (cam_point1(3)) + intrinsicMatrix(2,3)];%intrinsic_matrix * cam_point3';
    pixel_point4 = [intrinsicMatrix(1,1) * cam_point4(1) / (cam_point1(3)) + intrinsicMatrix(1,3), intrinsicMatrix(2,2) * cam_point4(2) / (cam_point1(3)) + intrinsicMatrix(2,3)];%intrinsic_matrix * cam_point4';
    pixel_point5 = [intrinsicMatrix(1,1) * cam_point5(1) / (cam_point1(3)) + intrinsicMatrix(1,3), intrinsicMatrix(2,2) * cam_point5(2) / (cam_point1(3)) + intrinsicMatrix(2,3)];%intrinsic_matrix * cam_point1';
    pixel_point6 = [intrinsicMatrix(1,1) * cam_point6(1) / (cam_point1(3)) + intrinsicMatrix(1,3), intrinsicMatrix(2,2) * cam_point6(2) / (cam_point1(3)) + intrinsicMatrix(2,3)];%intrinsic_matrix * cam_point2';
    pixel_point7 = [intrinsicMatrix(1,1) * cam_point7(1) / (cam_point1(3)) + intrinsicMatrix(1,3), intrinsicMatrix(2,2) * cam_point7(2) / (cam_point1(3)) + intrinsicMatrix(2,3)];%intrinsic_matrix * cam_point3';
    pixel_point8 = [intrinsicMatrix(1,1) * cam_point8(1) / (cam_point1(3)) + intrinsicMatrix(1,3), intrinsicMatrix(2,2) * cam_point8(2) / (cam_point1(3)) + intrinsicMatrix(2,3)];%intrinsic_matrix * cam_point4';
    pixel_point9 = [intrinsicMatrix(1,1) * cam_point9(1) / (cam_point1(3)) + intrinsicMatrix(1,3), intrinsicMatrix(2,2) * cam_point9(2) / (cam_point1(3)) + intrinsicMatrix(2,3)];%intrinsic_matrix * cam_point4';
    x_points = [pixel_point1(1) pixel_point2(1) pixel_point3(1) pixel_point4(1) pixel_point5(1) pixel_point6(1) pixel_point7(1) pixel_point8(1) pixel_point9(1)];
    y_points = [pixel_point1(2) pixel_point2(2) pixel_point3(2) pixel_point4(2) pixel_point5(2) pixel_point6(2) pixel_point7(2) pixel_point8(2) pixel_point9(2)];
    y_points = y_points;
    bbox_points = [x_points(:),y_points(:)];
    hasNaN = any(isnan(bbox_points(:)));
    hasNegative = any(bbox_points(:) < 0);
    if any(isnan(bbox_points), 2) 
        continue;
    end
    
    if any(isnan(bbox_points), 2) 
        continue;
    end
    
    bbox_points_undistorted = undistortPoints(bbox_points,intrinsics);
    bbox_points_undistorted(bbox_points_undistorted < 0) = 0;
    worldPoints = [cam_point1 cam_point2 cam_point3 cam_point4 cam_point5 cam_point6 cam_point7 cam_point8 cam_point9];
    pointsWithTheirProjection = [bbox_points_undistorted worldPoints(1:3,:)'];
    allData{j,5} = pointsWithTheirProjection;
    T = allData{j,3}(1:3,1:4);
    if any(isnan(T))
        continue;
    end
    PC = [PC1 ones(length(PC1),1)];
    masked_image = getMask(PC,T,intrinsicMatrix);
    se = strel('disk',5);
    BW_closed = imdilate(masked_image, se);
    allData{j,6} = BW_closed;
    allData{j,7} = intrinsicMatrix;
    allData{j,8} = T_dvs_r_base(1:3,1:4,2);
    obj_box = [p_1(1:3) p_2(1:3) p_3(1:3) p_4(1:3) p_5(1:3) p_6(1:3) p_7(1:3) p_8(1:3)];
end
%%
if ~exist(subDir, 'dir')
    mkdir(subDir);
end
threeChannelPathsFile = fullfile(subDir, 'three_channel_image_paths.txt');
pathsFileID = fopen(threeChannelPathsFile, 'w');

for i=1:length(allData)-5
    i
    bboxData = allData{i,5}(1:4,1:2); % Assuming bbox data is in the first column
    bboxDataRow = bboxData';
    bboxDataRow = bboxDataRow(:)';
    bboxFile = fullfile(subDir, sprintf('%04d_bbox.txt', i));
    fileID = fopen(bboxFile, 'w');
    fprintf(fileID, '%.4f ', bboxDataRow(1:end-1));
    fprintf(fileID, '%.4f\n', bboxDataRow(end));
    fclose(fileID);

    maskImageFile = fullfile(subDir, sprintf('%04d_mask_image.png', i));
    imwrite(allData{i,6}, maskImageFile);

    threeChannelImageFile = fullfile(subDir, sprintf('%04d_three_channel_image.png', i));
    imwrite(allData{i,2}, threeChannelImageFile);


    fprintf(pathsFileID, '%s\n', threeChannelImageFile);

    centerInfo = allData{i,5}(9,1:2); % Assuming center information is in the sixth column
    pose = allData{i,3} ; % Assuming pose is in the seventh column
    intrinsicMatrix = allData{i,7}; % Assuming intrinsic matrix is in the eighth column
    cameraPose = allData{i,8}; % Assuming camera pose is in the ninth column
    time = allData{i,4}; % Assuming time is in the tenth column
    keypointprojections =allData{i,5}(1:8,:); % Assuming 2D coordinates are in the eleventh column
    
    

    matFile = fullfile(subDir, sprintf('%04d_data.mat', i));
    save(matFile, 'centerInfo', 'pose', 'intrinsicMatrix', 'cameraPose', 'time', 'keypointprojections','cls_index');

end
fileID = fopen(ObjBoxFile, 'w');
for i = 1:size(obj_box, 1)
    fprintf(fileID, '%.4f ', obj_box(i, 1:end-1));
    fprintf(fileID, '%.4f\n', obj_box(i, end));
end
fclose(fileID);
clearvars -except fileNames folderPath cls_index T_obj_r_base fileList outputDir PC1 PC2 PC3
end
% %% Displaying data
% % Create a new figure window with a wider aspect ratio
% % Create a new figure window with a wider aspect ratio
% fig = figure;
% fig.Position = [100, 100, 1200, 400]; % [left bottom width height]
% 
% % Loop through each frame
% for i = 1:length(camera_position_upsampled)
% 
%     % --- 3D Transformation Visualization ---
%     subplot(1, 2, 1); % Select the subplot for 3D transformation
%     cla; % Clear the current axes
% 
%     % Set axis limits to 1m x 1m x 1m immediately after clearing the axes
%     xlim([-1 1]);
%     ylim([-1 1]);
%     zlim([-1 1]);    
% %     axis equal; % Maintain equal scaling
%     grid on; % Enable the grid
%     xlabel('X (m)');
%     ylabel('Y (m)');
%     zlabel('Z (m)');
%     hold on; % Retain plots after setting the axis
% 
%     % Base coordinate frame settings
%     arrowLength = 0.2; % Length of the arrows
% 
%     % Plot base coordinate frame (Identity transformation)
%     quiver3(0, 0, 0, arrowLength, 0, 0, 'r'); % X-axis
%     quiver3(0, 0, 0, 0, arrowLength, 0, 'g'); % Y-axis
%     quiver3(0, 0, 0, 0, 0, arrowLength, 'b'); % Z-axis
%     text(arrowLength, 0, 0, 'X', 'FontSize', 12, 'Color', 'r');
%     text(0, arrowLength, 0, 'Y', 'FontSize', 12, 'Color', 'g');
%     text(0, 0, arrowLength, 'Z', 'FontSize', 12, 'Color', 'b');
% 
%     % Rotation matrix and translation vector for the current frame
% %     R = rotm(:,:,i);
% %     t = camera_position_upsampled(i,:)';
% 
%     R = T_dvs_r_object(1:3,1:3,i);
%     t = T_dvs_r_object(1:3,4,i);
% 
%     % Convert rotation matrix to quaternion
%     quat = rotm2quat(R);
% 
%     % Plot the transformed coordinate frame using quaternion for rotation
%     plotTransforms(t', quat, 'FrameSize', 0.1); % Adjust 'FrameSize' as needed
%     title(sprintf('3D Transformation: Frame %d', i));
% 
%     % --- 2D Camera View Visualization ---
%     subplot(1, 2, 2); % Select the subplot for 2D camera view
%     cla; % Clear the current axes for 2D camera view
%     imshow(img{i,1})
%     xlim([0 346]); % Set X-axis limits based on camera resolution
%     ylim([0 260]); % Set Y-axis limits based on camera resolution
%     xlabel('X (pixels)');
%     ylabel('Y (pixels)');
%     title('Camera View');
%     drawnow;
% 
%     pause(0.000001); % Pause for a brief moment to create animation effect
% end
%%
% K = [289.182 0. 173.78;
%      0. 289.867  124.35;
%      0.  0.  1.];
% distCoeffs = [-0.38234 0.229166 0.001176 0.00069 -0.09235];
% radialDistortion = [-0.38234 0.229166];
% tangentialDistortion = [0.001176 0.00069];
% % tangentialDistortion = [0.0 0.0];
% cameraParams = cameraParameters('IntrinsicMatrix', K, ...
%                                 'RadialDistortion', radialDistortion, ...
%                                 'TangentialDistortion', tangentialDistortion);
% undistortedImageevent = undistortImage(img{1,1}, cameraParams);
%%
% Get image dimensions
% intrinsicMatrix = [289.182 0. 173.78;
%      0. 289.867  124.35;
%      0.  0.  1.];
% imagesize = [260 346];
% distCoeffs = [-0.38234 0.229166 0.001176 0.00069 -0.09235];
% intrinsics = cameraIntrinsicsFromOpenCV(intrinsicMatrix,...
%                                          distCoeffs,imagesize);
% I = img{300,1};
% J = undistortImage(I,intrinsics);

%%

