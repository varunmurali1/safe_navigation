%% Clean up
clc
clear all
close all

%% Load the map
image = imread('../Maps/simulations_floorplan_maze01.pgm');
imageBW = image < 100;
map = robotics.BinaryOccupancyGrid(imageBW,2)

% Display the map
figure(9);
show(map)
hold on;
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
prm.ConnectionDistance = 50;

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
figure(10);
show(prm)
hold on;

%% -------Start QP--------
x=zeros(1,3);

for i=1:(length(path)-1)
    
if (i==1)
    startX  = path(i,1);%3.0;
    startY  = path(i,2);%-6.0;
    phi     = 0.0;
    goalX   = path(i+1,1);%-3;
    goalY   = path(i+1,2);%-1;
    start_x = startX;
    start_y = startY;
else  
    startX  = goalX + X(501 ) %path(i,1);%3.0;
    startY  = goalY + Y(501)  %path(i,2);%-6.0;
    phi     = delta(length(Theta))- Theta(length(Theta));
    goalX   = path(i+1,1);%-3;
    goalY   = path(i+1,2);%-1;
    start_x = startX;
    start_y = startY;
end
 
% startX  = path(i,1);%3.0;
% startY  = path(i,2);%-6.0;
% phi     = 0.0;
% goalX   = path(i+1,1);%-3;
% goalY   = path(i+1,2);%-1;
%needed for final plotting
start_x = startX;
start_y = startY;
% Make this egocentric
startX  = startX - goalX;
startY  = startY - goalY;


%% Initialize the values
% Transform from cartesian to polar co-ordinates
% r         = sqrt(x^2 + y^2);
% theta     = atan2(y,x) - phi + pi
% delta     = gamma + phi

nx   = startX;
ny   = startY;
nt   = phi;

x(1)=sqrt(startX^2+startY^2);
%x(2)=atan2(startY,startX) - phi;
%x(3)=phi + x(2);
x(3)=wrapToPi(pi-atan2(startY,startX)) + phi;
x(2)=wrapToPi(-phi+ x(3));

%% Plot the start and the end state
r=0.5;
ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
figure(1);

plot(startX+xp,startY+yp,'r',[startX startX+r*cos(phi)],[startY startY+r*sin(phi)],'r','Linewidth',1.5)
hold on
plot(xp,yp,'b',[0 r],[0 0],'b','Linewidth',1.5)

%axis([-10 14 -10 14])
hold on

%% Initialize the values and storage variables
dT      = 0.01;
R       = [x(1)];
X       = [startX];
Y       = [startY];
Theta   = [];
k1      = 1;
k2      = 5;
Q       = diag([1, 0.1]);
delta   = [];
U1      = [];
U2      = [];
Slk     = [];
Slk2    = [];
head    = [];

k1      = 1;
k2      = 1;

%% Iterate over to find the optimal control
for i = 1:500
%x0=x(1)*sin(x(2));
%y0=x(1)*cos(x(2));

cvx_begin quiet
        variable u(4);
        minimize( u.' * u );
        subject to
            % Lyapunov function used:
            % V    = 0.5 * (r^2 + theta^2)
            % Derivative of V
            % Vdot = r*r_dot + theta*theta_dot
            %      = -r*v*cos(arctan(-k1*theta)) + (v/r) * theta *
            %      sin(arctan(-k1 * theta)

            -(x(1)) * u(1) * cos(atan(-k1*x(2))) + (x(2)) * u(1)/x(1) * sin(atan(-k1*x(2))) <= ...
                -0.5 * 2.0 * ((x(1))^2 + (x(2))^2) + u(3);
            z = x(3) - atan(-k1 * x(2));
            ((1 + 1/ (1 + x(2)^2) ) * u(1) / x(1) * sin(z + atan(-k1 * x(2))) + u(2)) == ...
                - k2 * u(1)/ x(1) * z + u(4);
            
            u(2) <= 0.2;
            u(2) >= -0.2;
            %u(1) <= 1
            
            % Barrier function candidate:
            % Measurement z = sqrt(r^2 + ro^2 - 2 * r * ro * cos(t - to) )
            % zdot = (1/z) * rdot * (r - cos(t-to)*ro) + (1/z) * tdot * (r * ro * sin(t - to))
            % Barrier
            % B(x,z) = 1 / (z - 0.5)
            % Bdot   = - zdot / (z - 0.5)^2
            
%             h    = x(1);
%             hdot = (1/h) * -1 * u(1) * cos(x(3)) *x(1); 
%             B    = 1 / (x(1) - 2.0);
%             Bdot = - hdot / (h - 2.0)^2;
%             
            %Bdot <= 1/B;
cvx_end

% Fixed control works
% u(1)=x(1);
% u(2)=-u(1)/x(1)*(k2*(x(3)-atan(-k1*x(2)))+1+k1/(1+(k1*x(2))^2)*sin(x(3)));


%% Smoothen the control
if size(U1,1) > 1
    u(1) = 0.85 * U1(end) + 0.15 * u(1);
    u(2) = 0.85 * U2(end) + 0.15 * u(2);
end

%% Propogate with dynamics
x(1) = x(1) - dT * u(1) * cos(x(3)); % + 0.5 * dT * u(1)/x(1) * sin(x(3)) + u(2));
x(2) = x(2) + dT * u(1)/x(1) * sin(x(3)); % + 0.5 * dT * u(1)/x(1) * sin(x(3)) + u(2));
x(3) = x(3) + dT * (u(1)/x(1) * sin(x(3))) + u(2);
x(3) = wrapToPi(x(3));
x(2) = wrapToPi(x(2));

nx   = nx + dT * u(1) * cos(nt + u(2)*dT/2);
ny   = ny + dT * u(1) * sin(nt + u(2)*dT/2);
nt   = nt + dT * u(2);

x

%% Store the variables
R=[R;x(1)];
Theta=[Theta;x(2)];
delta=[delta;x(3)];
X=[X; -x(1)*cos(x(2))];
Y=[Y; x(1)*sin(x(2))];
U1=[U1;u(1)];
U2=[U2;u(2)];
Slk = [Slk;u(3)];
Slk2= [Slk2;u(4)];
end

%% Generate plots
hold on;
figure(1); plot(X,Y); title('Trajectory')
%figure(9); plot(start_x +X,start_y+Y, 'r');
%hold on;
figure(9); plot(goalX +X,goalY+Y, 'g');
hold on;
% figure(2); plot(R); title('Distance to go')
% figure(3); plot(Theta); title('Theta')
% figure(4); plot(delta); title('Delta')
figure(5); plot(U1); title('U1')
figure(6); plot(U2); title('U2')
% figure(7); plot(Slk); title('Slk')

end



