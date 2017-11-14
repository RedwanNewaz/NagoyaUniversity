function map_view
clear all;clc;clf;
% % map=dlmread('map.txt',',');
load small_map
% % small_map=map(450:600,300:650);
c=contour(small_map);
% view(0,90)
% % save small_map.mat



path =dlmread('path.txt',',');
last_point=path(length(path),:);
hold on 
path = resamplePolygon(path, 500);
index=findClosestPoint(last_point,path)
path=2*path(1:index,:);
plot(path(:,1)+205,path(:,2)+50)
updated_path=[path(:,1)+205,path(:,2)+50];

R=2;
for i =1:5:length(updated_path)
    X=updated_path(i,:);
%     X=randi(151,1,2);
    drawCircle(X(:,1), X(:,2), R)
    pause(0.05)
    disp(measurement(X,small_map))
end
map=struct('map',small_map,'path',updated_path,'r_sz',2);
save map.mat

end

function [value]=measurement(pose,map)
pose=floor(pose);
value=map(pose(:,2),pose(:,1));
end