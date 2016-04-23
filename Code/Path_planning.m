%% Clean up
clc
clear all

%% Load the map
image = imread('../Maps/simulations_floorplan_maze01.pgm');
imageBW = image < 100;
map = robotics.BinaryOccupancyGrid(imageBW,2)

% Display the map
show(map)
robotRadius = 0.4;
mapInflated = copy(map);
inflate(mapInflated,robotRadius);

% Display inflated map
show(mapInflated)

%% Generate probabilistic roadmap to find waypoints
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
show(prm)

