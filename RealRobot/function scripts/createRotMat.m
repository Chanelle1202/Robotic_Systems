function [ rotMat ] = createRotMat(angle)
% Function which takes a 2D vector and returns a rotation matrix with 
% homogenous co-ordinates.

rotMat =[cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0;0 0 1];
end