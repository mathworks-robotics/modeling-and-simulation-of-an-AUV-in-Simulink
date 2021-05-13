function a = calc_ellipsemajoraxis(b,er,el)
% Calculate the major axis of an end-capped elliptical profile

theta = asin(er/b);
a = el/cos(theta);