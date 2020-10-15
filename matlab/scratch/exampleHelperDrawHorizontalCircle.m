function h = exampleHelperDrawHorizontalCircle(pos,r, c, ax)
%exampleHelperDrawHorizontalCircle

x = pos(1); y = pos(2); z = pos(3);
th = 0:pi/50:2*pi;
N = numel(th);
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
zunit = z * ones(1,N);
h = plot3(ax, xunit, yunit, zunit, c);

end