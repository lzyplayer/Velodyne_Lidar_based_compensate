function  anime_show(clouds,timestep)
%ANIME_SHOW 逐帧显示点云
%   clouds 点云组
%   timestep 播放帧间间隔（seconds），0为手动切换
axes = pcshow(clouds{1});
cscatter = axes.Children;
curr_cp = axes.CameraPosition;
curr_ct = axes.CameraTarget;
curr_cv = axes.CameraViewAngle;
for i=1:length(clouds)
    currPoints = clouds{i}.Location;
    cscatter.XData=currPoints(:,1);
    cscatter.YData=currPoints(:,2);
    cscatter.ZData=currPoints(:,3);
    cscatter.CData=currPoints(:,3);
    drawnow();
    axes.CameraPosition =  curr_cp;
    axes.CameraTarget   =  curr_ct;
    axes.CameraViewAngle = curr_cv;
    fprintf("%i  " ,i);
    if timestep==0
        input("show next?")
    else 
        pause(timestep);
        fprintf("\n" );
    end
    curr_cp = axes.CameraPosition;
    curr_ct = axes.CameraTarget;
    curr_cv = axes.CameraViewAngle;
end
end

