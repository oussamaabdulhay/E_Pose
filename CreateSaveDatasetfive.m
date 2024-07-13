clc
clear all
close all
%%
folderPath = 'D:/five_obj_clut/r_05_gl.bag';
fileList = dir(fullfile(folderPath, '*.bag'));
fileNames = {fileList.name};
PC1 = importdata('D:/five_obj_clut/domino.xyz');
PC2 = importdata('D:/five_obj_clut/drill.xyz');
PC3 = importdata('D:/five_obj_clut/mustard.xyz');
PC4 = importdata('D:/five_obj_clut/peg.xyz');
PC5 = importdata('D:/five_obj_clut/timer.xyz');

T_obj_r_base(:,:,1) =[-1.000 -0.05880 0.003879 -0.01472;
    0.058587 -0.9992 -0.04173 -0.80797;
    0.00633 -0.04151 0.999 0.0382;
    0 0 0 1];


T_obj_r_base(:,:,2)  = [0.9319 0.33058 0.15107 -0.0627;
    0.3624 -0.90445 -0.23041 -0.5616;
    0.06082 0.269 -0.95988 0.00749;
    0 0 0 1];

T_obj_r_base(:,:,3)  = [0.2972 0.95488 -0.0159 0.14106;
    -0.9548 0.2974 0.016118 -0.7411;
    0.02012 0.010418 0.9997 -0.0072;
    0 0 0 1];

T_obj_r_base(:,:,4)  = [0.29997 -0.03349 -0.9548 0.01871;
    -0.9544 0.0552 -0.3025 -0.7521;
    0.068355 0.9972 -0.1652 -0.01615;
    0 0 0 1];

T_obj_r_base(:,:,5) = [0.02973 0.6290 -0.78147 0.04079;
    -0.04072 0.7811 0.6264 -0.6441;
    1.000 0.00580 0.04380 0.030316;
    0 0 0 1];
obj1_name = 'domino';
obj2_name = 'drill';
obj3_name = 'mustard';
obj4_name = 'peg';
obj5_name = 'timer';
outputDir = 'D:/EYCB/five_obj_clut';
cls_index = [4 5 6 12 10];
for o = 1:length(fileNames)
subDir = fullfile(outputDir, fileNames{o}(1:end-4));
three_obj_file = fullfile(subDir, 'five_obj_clut.txt');
o
%%
bag_file_path = [folderPath, fileNames{o}];
bag=rosbag(bag_file_path);
bag.AvailableTopics;
%%
dvs_events_topic=select(bag,'Topic', '/dvs/events');  %, 'Time', time_range); 
dvs_events_msgs = readMessages(dvs_events_topic,'DataFormat','struct'); % topic & msg of DAVIS
dvs_pose_topic = select(bag, 'Topic', '/dvs/pose', ...);%,...
     'Time',[dvs_events_topic.StartTime dvs_events_topic.EndTime]); %, 'Time', time_range); 
dvs_pose = readMessages(dvs_pose_topic,'DataFormat','struct'); % topic & msg of DAVIS_pose
%% Assuming you have an estimate of the maximum size for each event array
results = cell(length(dvs_events_msgs), 1);

if isempty(gcp('nocreate'))
    parpool;
end

parfor i = 1:length(dvs_events_msgs)-1
    display(i)
    numEvents = length(dvs_events_msgs{i}.Events);
    temp_events = zeros(numEvents, 4); 

    for j = 1:numEvents
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
%% fixing the event times
camera_pose(:,8) = camera_pose(:,8) - events_parallel(1,4);
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

imagesize = [260 346];
distCoeffs = [-0.362 0.115 0.0009576 0.00000455 0.130];
intrinsicMatrix = [292.52 0.0 172.61;
                        0. 293.19  130.125;
                        0.  0.  1.];

intrinsics = cameraIntrinsicsFromOpenCV(intrinsicMatrix,...
                                         distCoeffs,imagesize);
for i=1:length(frames)
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
PC1 = PC1(:,1:3);
PC2 = PC2(:,1:3);
PC3 = PC3(:,1:3);
PC4 = PC4(:,1:3);
PC5 = PC5(:,1:3);
%%

rotm = quat2rotm(qi);
for i=1:length(camera_position_upsampled)
    T_dvs_r_base(:,:,i) = [rotm(:,:,i) camera_position_upsampled(i,:)'];
end
num_obj =size(T_obj_r_base(:,:,:));
for i=1:length(camera_position_upsampled)
    T_base_r_dvs (:,:,i) = inv([T_dvs_r_base(:,:,i);0 0 0 1]);
    for j = 1:num_obj(3)
        T_object_r_dvs(:,:,i,j) =  T_base_r_dvs (:,:,i) * T_obj_r_base(:,:,j);
        T_dvs_r_object(:,:,i,j) = inv([T_object_r_dvs(:,:,i,j)]);
    end

end
%% Putting everything in one data variable
for i=1:length(qi)
    allData{i,1} = frames{i,1};
    allData{i,2} = undistorted_frames{i,1};
    allData{i,3} = T_object_r_dvs(:,:,i,1);
    allData{i,4} = T_object_r_dvs(:,:,i,2);
    allData{i,5} = T_object_r_dvs(:,:,i,3);
    allData{i,6} = T_object_r_dvs(:,:,i,4);
    allData{i,7} = T_object_r_dvs(:,:,i,5);
    allData{i,8} = frames{i,2};
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

allData{j,9} = three_channel_image;
focalLength = [intrinsicMatrix(1,1), intrinsicMatrix(2,2)]; 
principalPoint = [intrinsicMatrix(1,3), intrinsicMatrix(2,3)]; 

% OBJ 1
R_test = allData{j,3}(1:3,1:3);
t_test = allData{j,3}(1:3,4);
p_1_obj1 = [max(PC1(:,1)) + 0.001;max(PC1(:,2))+ 0.001;min(PC1(:,3))+ 0.001;1];
p_2_obj1 = [max(PC1(:,1))+ 0.001;min(PC1(:,2))+ 0.001;min(PC1(:,3))+ 0.001;1];
p_3_obj1 = [min(PC1(:,1))+ 0.001;max(PC1(:,2))+ 0.001;min(PC1(:,3))+ 0.001;1];
p_4_obj1 = [min(PC1(:,1))+ 0.001;min(PC1(:,2))+ 0.001;min(PC1(:,3))+ 0.001;1];
p_5_obj1 = [max(PC1(:,1))+ 0.001;max(PC1(:,2))+ 0.001;max(PC1(:,3))+ 0.001;1];
p_6_obj1 =[max(PC1(:,1))+ 0.001;min(PC1(:,2))+ 0.001;max(PC1(:,3))+ 0.001;1];
p_7_obj1 = [min(PC1(:,1))+ 0.001;max(PC1(:,2))+ 0.001;max(PC1(:,3))+ 0.001;1];
p_8_obj1 =[min(PC1(:,1))+ 0.001;min(PC1(:,2))+ 0.001;max(PC1(:,3))+ 0.001;1];



p_9_obj1 = [0;0;0;1];

T_cam_r_obj = [R_test t_test;0 0 0 1];

cam_point1 = (T_cam_r_obj) * p_1_obj1 ;
cam_point2 =(T_cam_r_obj) * p_2_obj1 ;
cam_point3 = (T_cam_r_obj) * p_3_obj1 ;
cam_point4 = (T_cam_r_obj) * p_4_obj1 ;
cam_point5 = (T_cam_r_obj) * p_5_obj1 ;
cam_point6 =(T_cam_r_obj) * p_6_obj1 ;
cam_point7 = (T_cam_r_obj) * p_7_obj1 ;
cam_point8 = (T_cam_r_obj) * p_8_obj1 ;
cam_point9 = (T_cam_r_obj) * p_9_obj1 ;

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
allData{j,10} = pointsWithTheirProjection;


% OBJ 2
R_test = allData{j,4}(1:3,1:3);
t_test = allData{j,4}(1:3,4);
p_1_obj2 = [max(PC2(:,1)) + 0.001;max(PC2(:,2))+ 0.001;min(PC2(:,3))+ 0.001;1];
p_2_obj2 = [max(PC2(:,1))+ 0.001;min(PC2(:,2))+ 0.001;min(PC2(:,3))+ 0.001;1];
p_3_obj2 = [min(PC2(:,1))+ 0.001;max(PC2(:,2))+ 0.001;min(PC2(:,3))+ 0.001;1];
p_4_obj2 = [min(PC2(:,1))+ 0.001;min(PC2(:,2))+ 0.001;min(PC2(:,3))+ 0.001;1];
p_5_obj2 = [max(PC2(:,1))+ 0.001;max(PC2(:,2))+ 0.001;max(PC2(:,3))+ 0.001;1];
p_6_obj2 =[max(PC2(:,1))+ 0.001;min(PC2(:,2))+ 0.001;max(PC2(:,3))+ 0.001;1];
p_7_obj2 = [min(PC2(:,1))+ 0.001;max(PC2(:,2))+ 0.001;max(PC2(:,3))+ 0.001;1];
p_8_obj2 =[min(PC2(:,1))+ 0.001;min(PC2(:,2))+ 0.001;max(PC2(:,3))+ 0.001;1];



p_9_obj2 = [0;0;0;1];

T_cam_r_obj = [R_test t_test;0 0 0 1];

cam_point1 = (T_cam_r_obj) * p_1_obj2 ;
cam_point2 =(T_cam_r_obj) * p_2_obj2 ;
cam_point3 = (T_cam_r_obj) * p_3_obj2 ;
cam_point4 = (T_cam_r_obj) * p_4_obj2 ;
cam_point5 = (T_cam_r_obj) * p_5_obj2 ;
cam_point6 =(T_cam_r_obj) * p_6_obj2 ;
cam_point7 = (T_cam_r_obj) * p_7_obj2 ;
cam_point8 = (T_cam_r_obj) * p_8_obj2 ;
cam_point9 = (T_cam_r_obj) * p_9_obj2 ;

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
allData{j,11} = pointsWithTheirProjection;

% OBJ 3
R_test = allData{j,5}(1:3,1:3);
t_test = allData{j,5}(1:3,4);
p_1_obj3 = [max(PC3(:,1)) + 0.001;max(PC3(:,2))+ 0.001;min(PC3(:,3))+ 0.001;1];
p_2_obj3 = [max(PC3(:,1))+ 0.001;min(PC3(:,2))+ 0.001;min(PC3(:,3))+ 0.001;1];
p_3_obj3 = [min(PC3(:,1))+ 0.001;max(PC3(:,2))+ 0.001;min(PC3(:,3))+ 0.001;1];
p_4_obj3 = [min(PC3(:,1))+ 0.001;min(PC3(:,2))+ 0.001;min(PC3(:,3))+ 0.001;1];
p_5_obj3 = [max(PC3(:,1))+ 0.001;max(PC3(:,2))+ 0.001;max(PC3(:,3))+ 0.001;1];
p_6_obj3 =[max(PC3(:,1))+ 0.001;min(PC3(:,2))+ 0.001;max(PC3(:,3))+ 0.001;1];
p_7_obj3 = [min(PC3(:,1))+ 0.001;max(PC3(:,2))+ 0.001;max(PC3(:,3))+ 0.001;1];
p_8_obj3 =[min(PC3(:,1))+ 0.001;min(PC3(:,2))+ 0.001;max(PC3(:,3))+ 0.001;1];



p_9_obj3 = [0;0;0;1];

T_cam_r_obj = [R_test t_test;0 0 0 1];

cam_point1 = (T_cam_r_obj) * p_1_obj3 ;
cam_point2 =(T_cam_r_obj) * p_2_obj3 ;
cam_point3 = (T_cam_r_obj) * p_3_obj3 ;
cam_point4 = (T_cam_r_obj) * p_4_obj3 ;
cam_point5 = (T_cam_r_obj) * p_5_obj3 ;
cam_point6 =(T_cam_r_obj) * p_6_obj3 ;
cam_point7 = (T_cam_r_obj) * p_7_obj3 ;
cam_point8 = (T_cam_r_obj) * p_8_obj3 ;
cam_point9 = (T_cam_r_obj) * p_9_obj3 ;

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
allData{j,12} = pointsWithTheirProjection;


% OBJ 4
R_test = allData{j,6}(1:3,1:3);
t_test = allData{j,6}(1:3,4);
p_1_obj4 = [max(PC4(:,1)) + 0.001;max(PC4(:,2))+ 0.001;min(PC4(:,3))+ 0.001;1];
p_2_obj4 = [max(PC4(:,1))+ 0.001;min(PC4(:,2))+ 0.001;min(PC4(:,3))+ 0.001;1];
p_3_obj4 = [min(PC4(:,1))+ 0.001;max(PC4(:,2))+ 0.001;min(PC4(:,3))+ 0.001;1];
p_4_obj4 = [min(PC4(:,1))+ 0.001;min(PC4(:,2))+ 0.001;min(PC4(:,3))+ 0.001;1];
p_5_obj4 = [max(PC4(:,1))+ 0.001;max(PC4(:,2))+ 0.001;max(PC4(:,3))+ 0.001;1];
p_6_obj4 =[max(PC4(:,1))+ 0.001;min(PC4(:,2))+ 0.001;max(PC4(:,3))+ 0.001;1];
p_7_obj4 = [min(PC4(:,1))+ 0.001;max(PC4(:,2))+ 0.001;max(PC4(:,3))+ 0.001;1];
p_8_obj4 =[min(PC4(:,1))+ 0.001;min(PC4(:,2))+ 0.001;max(PC4(:,3))+ 0.001;1];



p_9_obj4 = [0;0;0;1];

T_cam_r_obj = [R_test t_test;0 0 0 1];

cam_point1 = (T_cam_r_obj) * p_1_obj4 ;
cam_point2 =(T_cam_r_obj) * p_2_obj4 ;
cam_point3 = (T_cam_r_obj) * p_3_obj4 ;
cam_point4 = (T_cam_r_obj) * p_4_obj4 ;
cam_point5 = (T_cam_r_obj) * p_5_obj4 ;
cam_point6 =(T_cam_r_obj) * p_6_obj4 ;
cam_point7 = (T_cam_r_obj) * p_7_obj4 ;
cam_point8 = (T_cam_r_obj) * p_8_obj4 ;
cam_point9 = (T_cam_r_obj) * p_9_obj4 ;

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
allData{j,13} = pointsWithTheirProjection;

% OBJ 5
R_test = allData{j,7}(1:3,1:3);
t_test = allData{j,7}(1:3,4);
p_1_obj5 = [max(PC5(:,1)) + 0.001;max(PC5(:,2))+ 0.001;min(PC5(:,3))+ 0.001;1];
p_2_obj5 = [max(PC5(:,1))+ 0.001;min(PC5(:,2))+ 0.001;min(PC5(:,3))+ 0.001;1];
p_3_obj5 = [min(PC5(:,1))+ 0.001;max(PC5(:,2))+ 0.001;min(PC5(:,3))+ 0.001;1];
p_4_obj5 = [min(PC5(:,1))+ 0.001;min(PC5(:,2))+ 0.001;min(PC5(:,3))+ 0.001;1];
p_5_obj5 = [max(PC5(:,1))+ 0.001;max(PC5(:,2))+ 0.001;max(PC5(:,3))+ 0.001;1];
p_6_obj5 =[max(PC5(:,1))+ 0.001;min(PC5(:,2))+ 0.001;max(PC5(:,3))+ 0.001;1];
p_7_obj5 = [min(PC5(:,1))+ 0.001;max(PC5(:,2))+ 0.001;max(PC5(:,3))+ 0.001;1];
p_8_obj5 =[min(PC5(:,1))+ 0.001;min(PC5(:,2))+ 0.001;max(PC5(:,3))+ 0.001;1];



p_9_obj5 = [0;0;0;1];

T_cam_r_obj = [R_test t_test;0 0 0 1];

cam_point1 = (T_cam_r_obj) * p_1_obj5 ;
cam_point2 =(T_cam_r_obj) * p_2_obj5;
cam_point3 = (T_cam_r_obj) * p_3_obj5 ;
cam_point4 = (T_cam_r_obj) * p_4_obj5 ;
cam_point5 = (T_cam_r_obj) * p_5_obj5 ;
cam_point6 =(T_cam_r_obj) * p_6_obj5 ;
cam_point7 = (T_cam_r_obj) * p_7_obj5 ;
cam_point8 = (T_cam_r_obj) * p_8_obj5 ;
cam_point9 = (T_cam_r_obj) * p_9_obj5 ;

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
allData{j,14} = pointsWithTheirProjection;


K = [292.52 0.0 172.61;
     0. 293.19  130.125;
     0.  0.  1.];

% tangentialDistortion = [0.0 0.0];
T1 = allData{j,3}(1:3,1:4);
T2 = allData{j,4}(1:3,1:4);
T3 = allData{j,5}(1:3,1:4);
T4 = allData{j,6}(1:3,1:4);
T5 = allData{j,7}(1:3,1:4);
PC1 = PC1(:,1:3);
PC2 = PC2(:,1:3);
PC3 = PC3(:,1:3);
PC4 = PC4(:,1:3);
PC5 = PC5(:,1:3);
PC1 = [PC1 ones(length(PC1),1)];
PC2 = [PC2 ones(length(PC2),1)];
PC3 = [PC3 ones(length(PC3),1)];
PC4 = [PC4 ones(length(PC4),1)];
PC5 = [PC5 ones(length(PC5),1)];
PCs_Ts{1,1} = PC1;
PCs_Ts{1,2} = T1;
PCs_Ts{2,1} = PC2;
PCs_Ts{2,2} = T2;
PCs_Ts{3,1} = PC3;
PCs_Ts{3,2} = T3;
PCs_Ts{4,1} = PC4;
PCs_Ts{4,2} = T4;
PCs_Ts{5,1} = PC5;
PCs_Ts{5,2} = T5;
masked_image = getMaskMultiple(PCs_Ts,K);
mask_obj_1 = (masked_image == 1);
mask_obj_2 = (masked_image == 2);
mask_obj_3 = (masked_image == 3);
mask_obj_4 = (masked_image == 4);
mask_obj_5 = (masked_image == 5);

% 
% se = strel('disk',20);
% BW_closed = imclose(masked_image, se);
allData{j,15} = allData{j,2}.*mask_obj_1;
allData{j,16} = allData{j,2}.*mask_obj_2;
allData{j,17} = allData{j,2}.*mask_obj_3;
allData{j,18} = allData{j,2}.*mask_obj_4;
allData{j,19} = allData{j,2}.*mask_obj_5;
allData{j,20} = masked_image;
allData{j,21} = mask_obj_1;
allData{j,22} = mask_obj_2;
allData{j,23} = mask_obj_3;
allData{j,24} = mask_obj_4;
allData{j,25} = mask_obj_5;
allData{j,26} = intrinsicMatrix;
allData{j,27} = T_dvs_r_base(1:3,1:4,2);
end
obj_1 = [p_1_obj1(1:3) p_2_obj1(1:3) p_3_obj1(1:3) p_4_obj1(1:3) p_5_obj1(1:3) p_6_obj1(1:3) p_7_obj1(1:3) p_8_obj1(1:3)];
obj_2 = [p_1_obj2(1:3) p_2_obj2(1:3) p_3_obj2(1:3) p_4_obj2(1:3) p_5_obj2(1:3) p_6_obj2(1:3) p_7_obj2(1:3) p_8_obj2(1:3)];
obj_3 = [p_1_obj3(1:3) p_2_obj3(1:3) p_3_obj3(1:3) p_4_obj3(1:3) p_5_obj3(1:3) p_6_obj3(1:3) p_7_obj3(1:3) p_8_obj3(1:3)];
obj_4 = [p_1_obj4(1:3) p_2_obj4(1:3) p_3_obj4(1:3) p_4_obj4(1:3) p_5_obj4(1:3) p_6_obj4(1:3) p_7_obj4(1:3) p_8_obj4(1:3)];
obj_5 = [p_1_obj5(1:3) p_2_obj5(1:3) p_3_obj5(1:3) p_4_obj5(1:3) p_5_obj5(1:3) p_6_obj5(1:3) p_7_obj5(1:3) p_8_obj5(1:3)];

%%
subDir = fullfile(outputDir, fileNames{o}(1:end-4));

if ~exist(subDir, 'dir')
    mkdir(subDir);
end
threeChannelPathsFile = fullfile(subDir, 'three_channel_image_paths.txt');
pathsFileID = fopen(threeChannelPathsFile, 'w');

for i=1:length(allData)-6
    i
    bboxData_obj1 = allData{i,10}(1:4,1:2); % Assuming bbox data is in the first column
    bboxDataRow_obj1 = bboxData_obj1';
    bboxDataRow_obj1 = bboxDataRow_obj1(:)';

    bboxData_obj2 = allData{i,11}(1:4,1:2); % Assuming bbox data is in the first column
    bboxDataRow_obj2 = bboxData_obj2';
    bboxDataRow_obj2 = bboxDataRow_obj2(:)';

    bboxData_obj3 = allData{i,12}(1:4,1:2); % Assuming bbox data is in the first column
    bboxDataRow_obj3 = bboxData_obj3';
    bboxDataRow_obj3 = bboxDataRow_obj3(:)';
    
    boxData_obj4 = allData{i,13}(1:4,1:2); % Assuming bbox data is in the first column
    bboxDataRow_obj4 = bboxData_obj3';
    bboxDataRow_obj4 = bboxDataRow_obj3(:)';
        
    bboxData_obj5 = allData{i,14}(1:4,1:2); % Assuming bbox data is in the first column
    bboxDataRow_obj5 = bboxData_obj3';
    bboxDataRow_obj5 = bboxDataRow_obj3(:)';

    bboxFile = fullfile(subDir, sprintf('%04d_bbox.txt', i));
    fileID = fopen(bboxFile, 'w');
    
    fprintf(fileID, '%s  ', obj1_name);
    fprintf(fileID, '%.4f ', bboxDataRow_obj1(1:end-1));
    fprintf(fileID, '%.4f\n', bboxDataRow_obj1(end));

    fprintf(fileID, '%s  ', obj2_name);
    fprintf(fileID, '%.4f ', bboxDataRow_obj2(1:end-1));
    fprintf(fileID, '%.4f\n', bboxDataRow_obj2(end));
    
    fprintf(fileID, '%s  ', obj3_name);
    fprintf(fileID, '%.4f ', bboxDataRow_obj3(1:end-1));
    fprintf(fileID, '%.4f\n', bboxDataRow_obj3(end));

    fprintf(fileID, '%s  ', obj4_name);
    fprintf(fileID, '%.4f ', bboxDataRow_obj4(1:end-1));
    fprintf(fileID, '%.4f\n', bboxDataRow_obj5(end));

    fprintf(fileID, '%s  ', obj5_name);
    fprintf(fileID, '%.4f ', bboxDataRow_obj5(1:end-1));
    fprintf(fileID, '%.4f\n', bboxDataRow_obj5(end));
    fclose(fileID);

    maskImageFile = fullfile(subDir, sprintf('%04d_mask_image_all.png', i));
    imwrite(allData{i,20}, maskImageFile);

    maskImageFile = fullfile(subDir, sprintf('%04d_mask_image_obj1.png', i));
    imwrite(allData{i,21}, maskImageFile);

    maskImageFile = fullfile(subDir, sprintf('%04d_mask_image_obj2.png', i));
    imwrite(allData{i,22}, maskImageFile);

    maskImageFile = fullfile(subDir, sprintf('%04d_mask_image_obj3.png', i));
    imwrite(allData{i,23}, maskImageFile);

    maskImageFile = fullfile(subDir, sprintf('%04d_mask_image_obj4.png', i));
    imwrite(allData{i,24}, maskImageFile);


    maskImageFile = fullfile(subDir, sprintf('%04d_mask_image_obj5.png', i));
    imwrite(allData{i,25}, maskImageFile);

    eventImageFile = fullfile(subDir, sprintf('%04d_event_image.png', i));
    imwrite(allData{i,2}, eventImageFile);

    threeChannelImageFile = fullfile(subDir, sprintf('%04d_three_channel_image.png', i));
    imwrite(allData{i,9}, threeChannelImageFile);

    segmentedImageFile = fullfile(subDir, sprintf('%04d_segmented_image_obj1.png', i));
    imwrite(allData{i,15}, segmentedImageFile);

    segmentedImageFile = fullfile(subDir, sprintf('%04d_segmented_image_obj2.png', i));
    imwrite(allData{i,16}, segmentedImageFile);

    segmentedImageFile = fullfile(subDir, sprintf('%04d_segmented_image_obj3.png', i));
    imwrite(allData{i,17}, segmentedImageFile);

    segmentedImageFile = fullfile(subDir, sprintf('%04d_segmented_image_obj4.png', i));
    imwrite(allData{i,18}, segmentedImageFile);

    segmentedImageFile = fullfile(subDir, sprintf('%04d_segmented_image_obj5.png', i));
    imwrite(allData{i,19}, segmentedImageFile);

    fprintf(pathsFileID, '%s\n', threeChannelImageFile);

    centerInfo = [allData{i,10}(9,1:2) allData{i,11}(9,1:2) allData{i,12}(9,1:2) allData{i,13}(9,1:2) allData{i,14}(9,1:2)]; % Assuming center information is in the sixth column
    pose = [allData{i,3} allData{i,4} allData{i,5} allData{i,6} allData{i,7}] ; % Assuming pose is in the seventh column
    intrinsicMatrix = allData{i,26}; % Assuming intrinsic matrix is in the eighth column
    cameraPose = allData{i,27}; % Assuming camera pose is in the ninth column
    time = allData{i,8}; % Assuming time is in the tenth column
    keypointprojections =[allData{i,10}(1:8,:) allData{i,11}(1:8,:) allData{i,12}(1:8,:) allData{i,13}(1:8,:) allData{i,14}(1:8,:)]; % Assuming 2D coordinates are in the eleventh column
    events =allData{i,1};
    

    matFile = fullfile(subDir, sprintf('%04d_data.mat', i));
    save(matFile, 'centerInfo', 'pose', 'intrinsicMatrix', 'cameraPose', 'time', 'keypointprojections', 'events','cls_index');

end
fileID = fopen(five_obj_file, 'w');
for i = 1:size(obj_1, 1)
    fprintf(fileID, '%.4f ', obj_1(i, 1:end-1));
    fprintf(fileID, '%.4f\n', obj_1(i, end));

    fprintf(fileID, '%.4f ', obj_2(i, 1:end-1));
    fprintf(fileID, '%.4f\n', obj_2(i, end));

    fprintf(fileID, '%.4f ', obj_3(i, 1:end-1));
    fprintf(fileID, '%.4f\n', obj_3(i, end));

    fprintf(fileID, '%.4f ', obj_4(i, 1:end-1));
    fprintf(fileID, '%.4f\n', obj_4(i, end));

    fprintf(fileID, '%.4f ', obj_5(i, 1:end-1));
    fprintf(fileID, '%.4f\n', obj_5(i, end));
end
fclose(fileID);
clearvars -except fileNames folderPath cls_index T_obj_r_base fileList outputDir PC1 PC2 PC3 PC4 PC5 obj1_name obj2_name obj3_name obj4_name obj5_name
end
%%
