function indicator =  anime_judge(clouds)

N=length(clouds);
indicator =  false(N,1);
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
    fprintf("current num : %i\n" ,i);
    curr_indi = input("press enter for OK, 0 for wrong ...");
    if isempty( curr_indi )
        indicator(i) = true;
    end
    fprintf("\n" );
    curr_cp = axes.CameraPosition;
    curr_ct = axes.CameraTarget;
    curr_cv = axes.CameraViewAngle;
end
end

