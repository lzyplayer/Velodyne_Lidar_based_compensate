function [rot] = yaw2rot(yaw)
%YAW2ROT 此处显示有关此函数的摘要
%   此处显示详细说明
sy = sin(yaw);
cy = cos(yaw);
rot=[cy -sy 0
    sy  cy  0
    0   0   1];
end

