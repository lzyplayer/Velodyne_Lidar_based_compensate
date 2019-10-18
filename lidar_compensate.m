function compensated_cloud = lidar_compensate(cloud,lidar_rate,velo_t,velo_yaw)
%LIDAR_COMPENSATE

sweep_time = 1/lidar_rate;
%cause lidar's 170degree is corvered, cannot calc lidar rota speed,use
%average instead
sweep_speed = 2*pi / sweep_time;
Location = cloud.Location;
%% yaw reset in order 
N = size(cloud.Location,1);
yaws=atan2( cloud.Location(:,2),cloud.Location(:,1));
start_yaw = max(yaws(1:16));
end_yaw = min(yaws(end-15:end));
if start_yaw-end_yaw>pi
else
    %s2 s3
    if (start_yaw>end_yaw ) || (end_yaw -start_yaw<pi)
        for i = N:-1:1
            if yaws(i)<end_yaw
                break;
            else
                yaws(i)=yaws(i)-2*pi;
            end
        end
    else
    %s4
    for i = N:-1:1

        if yaws(i)<end_yaw
            break;
        else
            yaws(i)=yaws(i)-4*pi;
        end
    end
    for j =1:N
        if yaws(j)>start_yaw
            break;
        end
    end
    yaws(j:i)=yaws(j:i)-2*pi;

    end
end
%% 
end_yaw = min(yaws(end-15:end));
yaw_range  = start_yaw-end_yaw;
%% process per point
for i=1:N
    

    time_passed = (start_yaw-yaws(i))/sweep_speed;
    R = eul2rotm([0,0,time_passed*velo_yaw],'XYZ');
    t = [velo_t*time_passed,0]';
    tMjo = [[R,t];[0,0,0,1]];
    
    p = [Location(i,:),1]';
    Location_t(i,:) = (tMjo*p)';
end
Location_t=Location_t(:,1:3);
compensated_cloud =pointCloud(Location_t);
end
