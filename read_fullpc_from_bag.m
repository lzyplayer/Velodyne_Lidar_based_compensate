function [clouds,stamps] = read_fullpc_from_bag(bagpath,if_timestamp)
%READ_FULLPC_FROM_BAG �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

    bag = rosbag(bagpath);
    %�����ȡ���ݰ����󱨴�ʱ��
    %1.����matlab java�ڴ��С
    %2.����step_times�������֣�Խ���ȡ�ٶ�Խ�ͣ�ռ��java�ڴ�Խ��
    step_times=4;  %��step_times����ȡ��������
    s= bag.StartTime;
    step= (bag.EndTime-bag.StartTime)/step_times;
    for i=1:step_times
        %ѡ�񼤹���ƻ���
        % robosense :  /rslidar_points
        % velodyne  :  /velodyne_points
        lidar_select = select(bag,'Topic','velodyne_points','Time',[s+(i-1)*step s+i*step]);
        if lidar_select.NumMessages==0
            error("please select correct topic");
        end
        rawClouds = readMessages(lidar_select);
        if nargin ==1
                clouds_team{i}=cellfun(@(msg) pointCloud(readXYZ(msg)),rawClouds,'UniformOutput',false);
        else
            if if_timestamp
                    clouds_team{i}=cellfun(@(msg) pointCloud(readXYZ(msg)),rawClouds,'UniformOutput',false);
                    clouds_stamp{i}=cellfun(@(msg) msg.Header.Stamp.seconds ,rawClouds,'UniformOutput',true);
%                 clouds_team=cellfun(@(msg) [{pointCloud(readXYZ(msg))},{msg.Header.Stamp.seconds}],rawClouds,'UniformOutput',false);
%                 clouds_store{i} =[ cellfun(@(x) x{1},clouds_team,'UniformOutput',false),cellfun(@(x) x{2},clouds_team,'UniformOutput',false)];
            end
        end
        clear lidar_select rawClouds ;
    end
    clouds={};stamps=[];
    for i=1:step_times
        clouds=[clouds;clouds_team{i}];
        stamps=[stamps;clouds_stamp{i}];
    end


end

