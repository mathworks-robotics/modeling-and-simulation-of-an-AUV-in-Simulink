%Plot Confidence Region

numPlat = 5;
distPlat = 6;
%offsetPlat  

th = 0:2*pi/numPlat:2*pi;
xunit = distPlat * cos(th(1:end-1));
yunit = distPlat * sin(th(1:end-1));

hold on
numPoints = 90;
r = linspace(0, 5, numPoints);                               % Define Radius & Radius Gradient Vector
a = linspace(0, 2*pi, numPoints);                             % Angles (Radians)
[R,A] = ndgrid(r, a);                                   % Create Grid
Z = -R;                                                 % Create Gradient Matrix
[X,Y,Z] = pol2cart(A,R,Z);                              % Convert To Cartesian
for i=1:1:length(xunit)
surf(X+xunit(i), Y+yunit(i), Z)
view(90,    90)
axis('equal')
colormap(flipud(autumn))
shading('interp')
end
