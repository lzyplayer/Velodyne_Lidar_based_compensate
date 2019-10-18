function [objects,ground] = remove_ground(clouds,z_trim)
%REMOVE_GROUND 此处显示有关此函数的摘要
%   此处显示详细说明
if nargin==1
    for i=1:length(clouds)
        maxDistance = 0.2;
        referenceVector = [0,0,1];
        maxAngularDistance = 10;
        [~,inlierIndices,outlierIndices] = pcfitplane(clouds{i},...
            maxDistance,referenceVector,maxAngularDistance,'Confidence',99);
        ground{i} = select(clouds{i},inlierIndices);
        objects{i} = select(clouds{i},outlierIndices);
        
    end
else
    for i=1:length(clouds)
        allInpoints = clouds{i}.Location;
        pointsGround = allInpoints(:,3)<z_trim;
        pointSelect = ~ pointsGround;%
        objects{i} = pointCloud(allInpoints(pointSelect,:));
        ground{i}=[];
    end
    
end
end

