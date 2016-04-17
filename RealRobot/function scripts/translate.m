function [ output ] = translate(pos,transMat )
% Funtion which takes in a homogenous translatation matrix 
% and a 2D position vector to perfrom a translation. 

posHomo = cat(2,pos,ones(size(pos,1),1));
output = (transMat*posHomo')';
output(:,3) = [];
end

