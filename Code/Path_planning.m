clc
clear all

image = imread('/Users/niharikaarora/Downloads/Maps/simulations_floorplan_maze01.pgm');
% a = size(image);
% imageBW = image;
% for i = 1:a(1)
%     for j = 1:a(2)
%         if image(i,j) >100
%             imageBW(i,j) = 1;
%         end
%     end
% end
%imshow(image)
%imshow(imageBW);
%imageBW = abs(imageBW - 1);
%imshow(imageBW);
imageBW = image < 100;


map = robotics.BinaryOccupancyGrid(imageBW,2)

% Display the map
show(map)
robotRadius = 0.4;
mapInflated = copy(map);
inflate(mapInflated,robotRadius);

% Display inflated map
show(mapInflated)

% Construct PRM and set parameters
prm = robotics.PRM

% Assign the inflated map to the PRM object
prm.Map = mapInflated;

% Number of nodes
prm.NumNodes = 1000;

% Distance between connections in visibility graph
prm.ConnectionDistance = 20;

% Find a feasible path on the constructed PRM
startLocation = [75 75];
endLocation = [250 250];

path = findpath(prm, startLocation, endLocation)

while isempty(path)
    % No feasible path found yet, increase the number of nodes
    prm.NumNodes = prm.NumNodes + 10;
    update(prm);
    path = findpath(prm, startLocation, endLocation);
end
% Display path
path

show(prm)

