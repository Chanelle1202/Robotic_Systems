function [ transMat ] = createTransMat( translation )
% Function which takes a 2D vector input and returns a translation matrix
% with homogenous co-ordinates.

transMat =[1 0 translation(1);0 1 translation(2);0 0 1];
end