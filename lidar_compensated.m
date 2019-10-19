% learn lidar compensated
%% read full clouds
clc;clear;close all;
disp("-----------starting read bagfile--------------");
bagpath = 'F:\rosbag_pointmap_fast\yuyao\yuyao_factory_8_7.bag';
bag = rosbag(bagpath);
[clouds,clouds_stamps] = read_fullpc_from_bag(bagpath,true);
disp("-----------bag file read complete--------------");
%%
% anime_show(clouds,0.01);
%%
% [cloud_structure,ground] = remove_ground(clouds);
% anime_show(cloud_structure,0);
%% init


lidar_rate=10;
sweep_time  = 1/lidar_rate;
sweep_speed = 2*pi/sweep_time;% init for situation which scan degree below 1.5pi
N= length(clouds);
clouds_CMPED{1}=clouds{1};
% compensate
a=2;b=N;
% a=380;b=450;
t_velosity(a-1,:)=[0,0];
yaw_velosity(a-1)=0;
clouds_CMPED{a-1}=clouds{a-1};
for i=a:b
    sweep_time = clouds_stamps(i)-clouds_stamps(i-1);
%     [clouds_CMPED{i},sweep_speed] = lidar_compensate(clouds{i},sweep_time,t_velosity(i-1,:),yaw_velosity(i-1),sweep_speed);
    [clouds_CMPED{i},sweep_speed] = lidar_compensate(clouds{i},sweep_time,[0 0],yaw_velosity(i-1),sweep_speed);
    Model = pcdownsample(clouds{i-1},'gridAverage',0.1);%_CMPED
    Data = pcdownsample(clouds{i},'gridAverage',0.1);%_CMPED
    [tform,moving_fixed] = pcregistericp( Data,Model, 'InlierRatio',0.85,'MaxIterations',50,'Tolerance',[1e-3,1e-3]); 
    t_velosity(i,:) = tform.T(4,1:2)./sweep_time;
%     pcshowpair(moving_fixed,Model)
    euler = rotm2eul(tform.T(1:3,1:3)','XYZ');
    if(abs(euler(1))>0.3)
        disp('inverse euler detected.')
        yaw_change = -euler(3);
    else
        yaw_change = euler(3);
    end
    yaw_velosity (i)= yaw_change/sweep_time;
    disp(i);
end
%%
figure
plot(yaw_velosity(a:b))
%%
anime_show(clouds_CMPED(a:b),0)
% anime_show(clouds(400:600),0.04)


%%
xlabel('x')
ylabel('y')
zlabel('z')