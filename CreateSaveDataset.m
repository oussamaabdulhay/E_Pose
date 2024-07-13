clc
clear all
close all
%%
folderPath = 'D:/data_collected/block/';
PC1 = importdata('D:/block/block.xyz');
outputDir = 'D:/E_pose/block';
fileList = dir(fullfile(folderPath, '*.bag'));
fileNames = {fileList.name};
T_obj_r_base=[0.9649 -0.2233 -0.1329 0.01097;
        0.1212 -0.03732 0.98912 -0.86408;
        -0.2293 -0.975808 -0.008539 0.026131;
         0 0 0 1];



cls_index = 1; 
for o = 4:length(fileNames)
% o = 16;
subDir = fullfile(outputDir, fileNames{o}(1:end-4));
ObjBoxFile = fullfile(subDir, 'block_box.txt');
if ~exist(subDir, 'dir')
    mkdir(subDir);
end
o
%%
bag_file_path = [folderPath, fileNames{o}];
bag=rosbag(bag_file_path);
bag.AvailableTopics;
%%
start_time = bag.StartTime;
end_time = bag.EndTime;
time_range_start = start_time;
time_range_end = end_time;
dvs_events_topic=select(bag,'Topic', '/dvs/events','Time', [time_range_start time_range_end]);  %, 'Time', time_range); 
dvs_events_msgs = readMessages(dvs_events_topic,'DataFormat','struct'); % topic & msg of DAVIS
dvs_pose_topic = select(bag, 'Topic', '/dvs/pose', ...);%,...
     'Time',[dvs_events_topic.StartTime dvs_events_topic.EndTime]); %, 'Time', time_range); 
dvs_pose = readMessages(dvs_pose_topic,'DataFormat','struct'); % topic & msg of DAVIS_pose
dvs_vel = select(bag, 'Topic', '/dvs/vel', ...);%,...
     'Time',[dvs_events_topic.StartTime dvs_events_topic.EndTime]); %, 'Time', time_range); 
dvs_vel_mgs  = readMessages(dvs_vel,'DataFormat','struct'); % topic & msg of DAVIS_pose
%% 
results = cell(length(dvs_events_msgs), 1);

if isempty(gcp('nocreate'))
    parpool;
end

parfor i = 1:length(dvs_events_msgs)-1
    display(i)
    numEvents = length(dvs_events_msgs{i}.Events);
    temp_events = zeros(numEvents, 4); 

    for j = 1:numEvents
%         j
        temp_events(j, 1) = double(dvs_events_msgs{i}.Events(j).X);
        temp_events(j, 2) = double(dvs_events_msgs{i}.Events(j).Y);
        temp_events(j, 3) = double(dvs_events_msgs{i}.Events(j).Polarity);

        if temp_events(j, 3) == 0
            temp_events(j, 3) = -1;
        end

        timeo = rostime(dvs_events_msgs{i}.Events(j).Ts.Sec, dvs_events_msgs{i}.Events(j).Ts.Nsec);
        temp_events(j, 4) = seconds(timeo);
    end

    results{i} = temp_events;
end

events_parallel = vertcat(results{:});
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
for i=1:length(dvs_vel_mgs)
    camera_vel(i,1) = dvs_vel_mgs{i}.Linear.X;
    camera_vel(i,2) = dvs_vel_mgs{i}.Linear.Y;
    camera_vel(i,3) = dvs_vel_mgs{i}.Linear.Z; 
    camera_vel(i,4) = dvs_vel.MessageList(i,1).Time;
end
%%
velo_mag = sqrt((camera_vel(:,1).^2)+(camera_vel(:,2).^2)+(camera_vel(:,3).^2));
%%
PC1 = PC1(:,1:3);
%%
events_parallel(:,4) = events_parallel(:,4) - 0.100;
%% fixing the event times
camera_pose(:,8) = camera_pose(:,8) - events_parallel(1,4);
camera_vel(:,4) = camera_vel(:,4) - events_parallel(1,4);
events_parallel(:,4) = events_parallel(:,4) - events_parallel(1,4);

%% removing all events before the first pose
cut_off = camera_pose(1,8);
events_parallel_ext = events_parallel;
cut_off_index = find(events_parallel_ext(:,4) >= cut_off, 1, 'first');
events_parallel_ext(1:cut_off_index,:)=[];
%% discretising events into frames
start_time = events_parallel_ext(1,4);
total_time = events_parallel_ext(end,4); 
step = 0.01; 

target_times = start_time:step:total_time;

indices_seg = zeros(size(target_times));

for i = 1:length(target_times)
    indices_seg(i) = find(events_parallel_ext(:,4) >= target_times(i), 1, 'first');
end

for i=2:length(indices_seg)
    frames{i-1,1} = [events_parallel_ext(indices_seg(i-1):indices_seg(i),1),(events_parallel_ext(indices_seg(i-1):indices_seg(i),2)),events_parallel_ext(indices_seg(i-1):indices_seg(i),4),events_parallel_ext(indices_seg(i-1):indices_seg(i),3)];
    frames{i-1,2} = events_parallel_ext(indices_seg(i),4);
    frames{i-1,3} = events_parallel_ext(indices_seg(i),3);
    frame_times(i-1)=events_parallel_ext(indices_seg(i),4);
end
%%
% for i =1:length(frames)
%     number_of_events_per_frame(i) = length(frames{i,1});
% end
% 
% 
% yy1 = smooth(frame_times,number_of_events_per_frame,0.1,'loess');
% yy2 = smooth(camera_vel(:,4),velo_mag*5000,0.001,'rloess');
% 
% plot(frame_times,yy1);
% hold on
% plot(camera_vel(:,4),yy2);
% [~, locs1] = findpeaks(-yy1);
% [~, locs2] = findpeaks(-yy2);
% time_diff = mean(frame_times(locs1)' - camera_vel(locs2,4));
% % events_parallel(:,4) = events_parallel(:,4) - 0.058;

%%

imagesize = [260 346];
distCoeffs = [-0.362 0.115 0.0009576 0.00000455 0.130];
intrinsicMatrix = [292.52 0.0 172.61;
                        0. 293.19  130.125;
                        0.  0.  1.];

intrinsics = cameraIntrinsicsFromOpenCV(intrinsicMatrix,...
                                         distCoeffs,imagesize);
for i=1:length(frames)
    i
    event_img = zeros(260, 346);
    evs = frames{i,1}(:,:);
    for k=1:length(evs)
        if (evs(k,2) <= 0) | (evs(k,1) <= 0)
            continue
        else
            event_img(evs(k,2),evs(k,1)) = 1;
        end
    end
    I = event_img;
    undistorted_frames{i,1} = undistortImage(I,intrinsics);
end

%% interpolating pose data
for i=1:length(frames)
    pose_before = find(camera_pose(:,8) <= frames{i,2} , 1, 'Last');
    pose_after = find(camera_pose(:,8) > frames{i,2} , 1, 'first');
    
    if pose_after ~= 1
        time_before = camera_pose(pose_before,8);
        time_after = camera_pose(pose_after,8);
        time_current = frames{i,2};
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

rotm = quat2rotm(qi);
for i=1:length(camera_position_upsampled)
    T_dvs_r_base(:,:,i) = [rotm(:,:,i) camera_position_upsampled(i,:)'];
end
for i=1:length(camera_position_upsampled)
    T_base_r_dvs (:,:,i) = inv([T_dvs_r_base(:,:,i);0 0 0 1]);
    T_object_r_dvs(:,:,i) =  T_base_r_dvs (:,:,i) * T_obj_r_base;
    T_dvs_r_object(:,:,i) = inv([T_object_r_dvs(:,:,i)]);
end
%% Putting everything in one data variable
for i=1:length(qi)
    allData{i,1} = frames{i,1};
    allData{i,2} = undistorted_frames{i,1};
    allData{i,3} = T_object_r_dvs(:,:,i);
    allData{i,4} = frames{i,2};
end
%%
pos_ts = zeros(260,346);
neg_ts = zeros(260,346);
event_ts = zeros(260,346);
for j=1:length(allData)
    j
    for i=1:length(frames{j})
        if frames{j}(i,2) < 1 || frames{j}(i,1)<1 
            continue;
        end
        event_ts (frames{j}(i,2),frames{j}(i,1)) = event_ts (frames{j}(i,2),frames{j}(i,1)) + frames{j}(i,3);
        if frames{j}(i,4) > 0
            pos_ts (frames{j}(i,2),frames{j}(i,1)) = frames{j}(i,3);
        elseif frames{j}(i,4) < 0
            neg_ts (frames{j}(i,2),frames{j}(i,1)) = frames{j}(i,3);
        end
    end
event_ts = mat2gray(event_ts);
pos_ts = mat2gray(pos_ts);
neg_ts = mat2gray(neg_ts);


event_ts_undistorted = undistortImage(event_ts,intrinsics);
pos_ts_undistorted = undistortImage(pos_ts,intrinsics);
neg_ts_undistorted = undistortImage(neg_ts,intrinsics);


three_channel_image = cat(3, event_ts_undistorted, pos_ts_undistorted, neg_ts_undistorted);

allData{j,5} = three_channel_image;

R_test = allData{j,3}(1:3,1:3);
t_test = allData{j,3}(1:3,4);
focalLength = [intrinsicMatrix(1,1), intrinsicMatrix(2,2)]; 
principalPoint = [intrinsicMatrix(1,3), intrinsicMatrix(2,3)]; 
p_1 = [max(PC1(:,1)) ;max(PC1(:,2));min(PC1(:,3));1];
p_2 = [max(PC1(:,1));min(PC1(:,2));min(PC1(:,3));1];
p_3 = [min(PC1(:,1));max(PC1(:,2));min(PC1(:,3));1];
p_4 = [min(PC1(:,1));min(PC1(:,2));min(PC1(:,3));1];
p_5 = [max(PC1(:,1));max(PC1(:,2));max(PC1(:,3));1];
p_6 =[max(PC1(:,1));min(PC1(:,2));max(PC1(:,3));1];
p_7 = [min(PC1(:,1));max(PC1(:,2));max(PC1(:,3));1];
p_8 =[min(PC1(:,1));min(PC1(:,2));max(PC1(:,3));1];


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
allData{j,6} = pointsWithTheirProjection;


T = allData{j,3}(1:3,1:4);
if any(isnan(T))
    continue;
end
PC = [PC1 ones(length(PC1),1)];
masked_image = getMask(PC,T,intrinsicMatrix);
se = strel('disk',3);
BW_closed = imdilate(masked_image, se);
allData{j,7} = allData{j,2}.*BW_closed;
allData{j,8} = BW_closed;
allData{j,9} = intrinsicMatrix;
allData{j,10} = T_dvs_r_base(1:3,1:4,2);
end
obj_box = [p_1(1:3) p_2(1:3) p_3(1:3) p_4(1:3) p_5(1:3) p_6(1:3) p_7(1:3) p_8(1:3)];
%%

if ~exist(subDir, 'dir')
    mkdir(subDir);
end
threeChannelPathsFile = fullfile(subDir, 'three_channel_image_paths.txt');
pathsFileID = fopen(threeChannelPathsFile, 'w');

for i=1:length(allData)-12
    i
    bboxData = allData{i,6}(1:4,1:2); % Assuming bbox data is in the first column
    bboxDataRow = bboxData';
    bboxDataRow = bboxDataRow(:)';
    bboxFile = fullfile(subDir, sprintf('%04d_bbox.txt', i));
    fileID = fopen(bboxFile, 'w');
    fprintf(fileID, '%.4f ', bboxDataRow(1:end-1));
    fprintf(fileID, '%.4f\n', bboxDataRow(end));
    fclose(fileID);

    maskImageFile = fullfile(subDir, sprintf('%04d_mask_image.png', i));
    imwrite(allData{i,8}, maskImageFile);

    eventImageFile = fullfile(subDir, sprintf('%04d_event_image.png', i));
    imwrite(allData{i,2}, eventImageFile);

    threeChannelImageFile = fullfile(subDir, sprintf('%04d_three_channel_image.png', i));
    imwrite(allData{i,5}, threeChannelImageFile);

    segmentedImageFile = fullfile(subDir, sprintf('%04d_segmented_image.png', i));
    imwrite(allData{i,7}, segmentedImageFile);

    fprintf(pathsFileID, '%s\n', threeChannelImageFile);

    centerInfo = allData{i,6}(9,1:2); % Assuming center information is in the sixth column
    pose = allData{i,3} ; % Assuming pose is in the seventh column
    intrinsicMatrix = allData{i,9}; % Assuming intrinsic matrix is in the eighth column
    cameraPose = allData{i,10}; % Assuming camera pose is in the ninth column
    time = allData{i,4}; % Assuming time is in the tenth column
    keypointprojections =allData{i,6}(1:8,:); % Assuming 2D coordinates are in the eleventh column
    events =allData{i,1};
    
    

    matFile = fullfile(subDir, sprintf('%04d_data.mat', i));
    save(matFile, 'centerInfo', 'pose', 'intrinsicMatrix', 'cameraPose', 'time', 'keypointprojections', 'events','cls_index');

end
fileID = fopen(ObjBoxFile, 'w');
for i = 1:size(obj_box, 1)
    fprintf(fileID, '%.4f ', obj_box(i, 1:end-1));
    fprintf(fileID, '%.4f\n', obj_box(i, end));
end
fclose(fileID);
clearvars -except fileNames folderPath cls_index T_obj_r_base fileList outputDir PC1 PC2 PC3
end
