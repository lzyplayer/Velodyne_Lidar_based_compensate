function  anime_show(clouds,timestep)
%ANIME_SHOW ��֡��ʾ����
%   clouds ������
%   timestep ����֡������seconds����0Ϊ�ֶ��л�
axes = pcshow(clouds{1});
view(2);
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

