function h = plotCircle(r)
figure
hold on
th = 0:2*pi/4:2*pi;
xunit = r * cos(th(1:end-1));
yunit = r * sin(th(1:end-1));
h = plot(xunit, yunit);

% colored following x value and current colormap
hold off