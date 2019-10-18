% construct lidar map 
%% read full clouds
clc;clear;close all;
disp("-----------starting read bagfile--------------");
%rosbag���ݰ�·�� rosbag��Ҫ�����������״���Ϣ��������̼���Ϣ
bagpath = '3dbag_0921_1047.bag';
%�����º������޸ļ����״ﻰ��
[clouds,clouds_stamps] = read_fullpc_from_bag(bagpath,true);
%�����º������޸�������̼ƻ���
motion_stack = Motion_stack_from_odom(bagpath);
disp("-----------bag file read complete--------------");
%% select pc by step
select_step=8;%����ѡȡ���������С����ٶ��趨
select_pc = clouds(1:select_step:end,:);
select_stamps = clouds_stamps(1:select_step:end,:);
%% iter check(optional in compensated data)
%show pointcloud by frame 
%press enter stands for correct data
%press '0' and enter stands for wrong data
indicator = anime_judge(select_pc);
select_pc=select_pc(indicator);
select_stamps = select_stamps(indicator);
close;
%% trim ground point cloud
disp("-----------removing ground--------------");
[test_clouds,ground_clouds] = remove_ground(select_pc);
%��������С����ʱ����ע������˳��㷨������
clouds_ng = test_clouds;
anime_show(ground_clouds,0.1);close;
%% pc pre-regis trim
%���Ʋü�
trclouds = pcTrim(clouds_ng,0.5,50,[-1,8],0.03);
disp("-----------cloud trim completed--------------");
%% centerlise clouds
%������ͼ����
[centered_clouds,centered_motion,centered_stamps] = pc_centerlise_with_wheel(trclouds,select_stamps,motion_stack,15,0.03);
%% downsample
for i=1:length(centered_clouds)
    downsample_centers{i} = pcdownsample(centered_clouds{i},'gridAverage',0.03);
end
%% slam
%���ټ����ں�  SLAM
MotionGlobal = slam_with_wheel(downsample_centers,centered_stamps,motion_stack);
%% result display 
%չʾSLAM�����pcd����
result_cloud  =obtainResult(downsample_centers,MotionGlobal,true,0.03);
pcwrite(result_cloud,'wuhan_velodyne.pcd','Encoding','ascii');
disp("-----------pcd saved--------------");