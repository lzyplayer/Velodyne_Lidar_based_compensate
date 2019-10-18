% learn lidar compensated
%% read full clouds
clc;clear;close all;
disp("-----------starting read bagfile--------------");
bagpath = '3dbag_0921_1047.bag';
bag = rosbag(bagpath);
[clouds,clouds_stamps] = read_fullpc_from_bag(bagpath,true);
disp("-----------bag file read complete--------------");
%%
% anime_show(clouds,0.01);
%%
[cloud_structure,ground] = remove_ground(clouds);
anime_show(ground,0.01);
%% init
t_velosity=[0,0];
yaw_velosity=0;
lidar_rate=10;
sweep_time  = 1/lidar_rate;
N= length(clouds);
clouds_CMPED{1}=clouds{1};
%% compensate
clouds_CMPED{609}=clouds{609};
for i=610:620
    clouds_CMPED{i} = lidar_compensate(clouds{i},lidar_rate,t_velosity,yaw_velosity);
    [tform,moving_fixed] = pcregistericp(clouds_CMPED{i},clouds_CMPED{i-1}, 'InlierRatio',0.85,'MaxIterations',50,'Tolerance',[1e-3,1e-3]);
    t_velosity = tform.T(4,1:2)./sweep_time;
%     pcshowpair(moving_fixed,clouds_CMPED{i-1})
    euler = rotm2eul(tform.T(1:3,1:3)','XYZ');
    if(abs(euler(1))>0.3)
        yaw_change = -eluer(3);
    else
        yaw_change = euler(3);
       
%         if i==612
%             1==1;
%         end
        
    end
    yaw_velosity = yaw_change/sweep_time;
    disp(i);
end
%% registration


%%
xlabel('x')
ylabel('y')
zlabel('z')