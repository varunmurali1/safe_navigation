%% Clean up
close all
clear;
clc;

%% Generate starting points
x=zeros(1,3);
startx  =-7;
starty  =12;
phi     =0.0;

%% Initialize the values
% Transform from cartesian to polar co-ordinates
% r         = sqrt(x^2 + y^2);
% theta     = atan2(y,x) - phi + pi
% delta     = gamma + phi
%x(1) = sqrt(startx^2 + starty^2);
%x(2) = atan2(starty, startx) - phi + pi;
%x(3) = x(2) + phi;
nx   = startx;
ny   = starty;
nt   = phi;

x(1)=sqrt(startx^2+starty^2);
x(2)=atan2(starty,startx);
x(3)=phi;

%% Plot the start and the end state
r=0.5;
ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
figure(1);

plot(startx+xp,starty+yp,'r',[startx startx+r*cos(phi)],[starty starty+r*sin(phi)],'r','Linewidth',1.5)
hold on
plot(xp,yp,'b',[0 r],[0 0],'b','Linewidth',1.5)

axis([-10 14 -10 14])
hold on

%% Initialize the values and storage variables
dT      = 0.01;
R       = [];
X       = [startx];
Y       = [starty];
Theta   = [];
k1      = 1;
k2      = 1;
Q       = diag([1, 0.1]);
delta   = [];
U1      = [];
U2      = [];
Slk     = [];
Slk2    = [];
k1      = 1;
k2      = 1;

%% Iterate over to find the optimal control
for i = 1:300
x0=x(1)*sin(x(2));
y0=x(1)*cos(x(2));

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

            -x(1) * u(1) * cos(atan(-k1*x(2))) + u(1)/x(1) *x(2)* sin(atan(-k1*x(2))) <= ...
                -0.5 * 2.0 * (x(1)^2 + x(2)^2) + u(3);
            z = x(3) - atan(-x(2));
            ((1 + 1/ (1 + x(2)^2) ) * u(1) / x(1) * sin(z + atan(-x(2))) + u(2)) == ...
                -0.5 * u(1)/ x(1) * z + u(4);
            
            % Barrier function candidate:
            % Measurement z = sqrt(r^2 + ro^2 - 2 * r * ro * cos(t - to) )
            % zdot = (1/z) * rdot * (r - cos(t-to)*ro) + (1/z) * tdot * (r * ro * sin(t - to))
            % Barrier
            % B(x,z) = 1 / (z - 0.5)
            % Bdot   = - zdot / (z - 0.5)^2
            
            h    = x(1);
            hdot = (1/h) * -1 * u(1) * cos(x(3)) *x(1); 
            B    = 1 / (x(1) - 2.0);
            Bdot = - hdot / (h - 2.0)^2;
            
            Bdot <= 1/B;
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
x(1) = x(1) - dT * u(1) * cos(x(3));
x(2) = x(2) + dT * u(1)/x(1) * sin(x(3));
x(3) = x(3) + dT * u(1)/x(1) * sin(x(3)) + u(2);

nx   = nx + dT * u(1) * cos(nt);
ny   = ny + dT * u(1) * sin(nt);
nt   = nt + dT * u(2);

%% Store the variables
R=[R;x(1)];
Theta=[Theta;x(2)];
delta=[delta;x(3)];
X=[X;x(1)*cos(x(2))];
Y=[Y;x(1)*sin(x(2))];
U1=[U1;u(1)];
U2=[U2;u(2)];
Slk = [Slk;u(3)];
Slk2= [Slk2;u(4)];
end

%% Generate plots
hold on;
figure(1); plot(X,Y); title('Trajectory')
figure(2); plot(R); title('Distance to go')
figure(3); plot(Theta); title('Theta')
figure(4); plot(delta); title('Delta')
figure(5); plot(U1); title('U1')
figure(6); plot(U2); title('U2')
figure(7); plot(Slk); title('Slk')
