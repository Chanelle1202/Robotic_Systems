function [ crossingPoint] = intersection(infVec1,line2)
% Function which calculates the intersecting point between an infinite 
% vector and a bounded line. 

ua = ((line2(:,3)-line2(:,1)).*(infVec1(:,2)-line2(:,2))-(line2(:,4)-line2(:,2)).*(infVec1(:,1)-line2(:,1)))./(((line2(:,4)-line2(:,2)).*(infVec1(:,3)-infVec1(:,1))-(line2(:,3)-line2(:,1)).*(infVec1(:,4)-infVec1(:,2))));
ub = ((infVec1(:,3)-infVec1(:,1)).*(infVec1(:,2)-line2(:,2))-(infVec1(:,4)-infVec1(:,2)).*(infVec1(:,1)-line2(:,1)))./(((line2(:,4)-line2(:,2)).*(infVec1(:,3)-infVec1(:,1))-(line2(:,3)-line2(:,1)).*(infVec1(:,4)-infVec1(:,2))));
filter = ub >= 0 & ub <= 1 & ua>=0; %
crossingPoint =[infVec1(:,1)+ua.*(infVec1(:,3)-infVec1(:,1)) infVec1(:,2)+ua.*(infVec1(:,4)-infVec1(:,2))];
for i = 1:length(filter)
    if filter(i) == 0
        crossingPoint(i,:) = NaN(1,2);
    end
end
end