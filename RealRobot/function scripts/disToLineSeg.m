function [ distances ] = disToLineSeg(point,line )
% Function which calculates and returns the shortest distance between a 
% a given point and line segment.

lStart = line(:,[1 2]); % line start point
lEnd = line(:,[3 4]); % line end point
point = repmat(point,size(line,1),1);
vec = lEnd - lStart;
lenSQ = sum(vec.^2,2);
t = dot(point-line(:,[1 2]),vec,2)./lenSQ;
distances = zeros(length(t),1);
for i=1:length(t)
    if t(i) <0
        distances(i) = distance(point(i,:),lStart(i,:));
    elseif t(i) >1
        distances(i) = distance(point(i,:),lEnd(i,:));
    else
        projection = lStart(i,:) + t(i) * (vec(i,:));
        distances(i) = distance(point(i,:), projection);
    end
end
end
