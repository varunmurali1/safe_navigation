
% use this to generate 8 evenly spaced starting positions on a circle

num=8;
startX=zeros(8,1);
startY=zeros(8,1);
rad=8;
for i=1:num
    angle=2*pi/num*i;
    startX(i)=rad*sin(angle);
    startY(i)=rad*cos(angle);
end

% change iter to num